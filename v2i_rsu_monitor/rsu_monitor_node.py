#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 RSU SAE J2735 Monitor (MAP + SPaT + SDSM) — subscribes to topics instead of UDP
Topics expected (message type: std_msgs/UInt8MultiArray):
- v2i/sdsm/raw  -> SDSM
- v2i/map/raw   -> MAP
- v2i/spat/raw  -> SPAT

Run via: `ros2 run v2i_rsu_monitor rsu_monitor`
"""

import sys
import math
import os
import time
import threading
import queue
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from pycmssdk.asn1 import Asn1Type, asn1_decode
from PyQt5 import QtWidgets, QtCore, QtGui, QtNetwork

try:
    from v2i_sdsm_msgs.msg import SDSM
except ImportError:
    SDSM = None

try:
    from v2i_spat_msgs.msg import SpatPacket
except ImportError:
    SpatPacket = None

# -------------------------
# CONFIG (copied from original)
# -------------------------
UDP_IP = "0.0.0.0"

PORTS = {
    7111: "ALL",
    7112: "SDSM",
    7113: "MAP",
    7114: "SPAT",
}

HEX_PREFIX = {
    "MAP": "0012",
    "SPAT": "0013",
    "SDSM": "0029",
}

GUI_UPDATE_HZ = 10

TARGET_ORIGIN_NAME = "MLK/GEORGIA"
TARGET_ORIGIN_LAT = 35.045777
TARGET_ORIGIN_LON = -85.3082791111

LOCK_MAP_AFTER_FIRST = True
FREEZE_UI_AFTER_FIRST_MAP = True

RSU_CALIB = {}
DEFAULT_CALIB = (0.0, 0.0, 0.0)

MAP_XY_UNIT_METERS = 0.01
SDSM_OFFSET_UNIT_METERS = 0.1
SDSM_OFFSET_AXES_DEFAULT = "EN"
SDSM_OFFSET_AXES_BY_SENDER = {}
SDSM_OFFSET_YAW_DEG_DEFAULT = 0.0
SDSM_OFFSET_YAW_DEG_BY_SENDER = {}

PIXELS_PER_METER = 4.0
DOT_RADIUS_PX = 5

SCENE_PAD_M = 30.0
MAX_SCENE_HALF_SIZE_M = 250.0
INTERSECTION_ZOOM_PAD_M = 20.0

EARTH_RADIUS_M = 6378137.0

OSM_ENABLED_DEFAULT = True
OSM_TILE_URL_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSM_TILE_ZOOM = 18
OSM_MIN_TILE_ZOOM = 14
OSM_MAX_TILES = 96
OSM_USER_AGENT = "v2i_rsu_monitor/0.1 local ROS2 monitor"
OSM_CACHE_DIR = os.path.expanduser("~/.cache/v2i_rsu_monitor/osm_tiles")
OSM_TILE_Z_VALUE = -100.0


# -------------------------
# UTILS (copied)
# -------------------------
def now_iso():
    return datetime.utcnow().isoformat(timespec="milliseconds") + "Z"


def apply_rigid_transform_xy(x, y, dx, dy, yaw_deg):
    th = math.radians(yaw_deg)
    c = math.cos(th)
    s = math.sin(th)
    xr = c * x - s * y
    yr = s * x + c * y
    return xr + dx, yr + dy


def rotate_xy(x, y, yaw_deg):
    th = math.radians(yaw_deg)
    c = math.cos(th)
    s = math.sin(th)
    return c * x - s * y, s * x + c * y


def apply_offset_axes(ox_m, oy_m, mode):
    m = (mode or "EN").strip()
    if m == "EN":
        return ox_m, oy_m
    if m == "NE":
        return oy_m, ox_m
    if m == "E_minusN":
        return ox_m, -oy_m
    if m == "minusE_N":
        return -ox_m, oy_m
    if m == "minusE_minusN":
        return -ox_m, -oy_m
    return ox_m, oy_m


def get_sdsm_alignment_for_sender(sender):
    axes_mode = SDSM_OFFSET_AXES_BY_SENDER.get(sender, SDSM_OFFSET_AXES_DEFAULT)
    yaw_deg = SDSM_OFFSET_YAW_DEG_BY_SENDER.get(sender, SDSM_OFFSET_YAW_DEG_DEFAULT)
    return axes_mode, yaw_deg


def find_first_key_recursive(obj, candidate_keys):
    if isinstance(obj, dict):
        for k in candidate_keys:
            if k in obj:
                return k, obj[k]
        for v in obj.values():
            fk, fv = find_first_key_recursive(v, candidate_keys)
            if fk is not None:
                return fk, fv
    elif isinstance(obj, (list, tuple)):
        for it in obj:
            fk, fv = find_first_key_recursive(it, candidate_keys)
            if fk is not None:
                return fk, fv
    return None, None


def find_all_matches_recursive(obj, match_fn, results=None):
    if results is None:
        results = []
    if match_fn(obj):
        results.append(obj)
    if isinstance(obj, dict):
        for v in obj.values():
            find_all_matches_recursive(v, match_fn, results)
    elif isinstance(obj, (list, tuple)):
        for item in obj:
            find_all_matches_recursive(item, match_fn, results)
    return results


def normalize_name(name_val):
    if name_val is None:
        return None
    if isinstance(name_val, str):
        s = name_val.strip()
        return s if s else None
    if isinstance(name_val, (bytes, bytearray)):
        try:
            s = name_val.decode("utf-8", errors="ignore").strip()
            return s if s else None
        except Exception:
            s = str(name_val).strip()
            return s if s else None
    if isinstance(name_val, dict):
        for k in ["text", "value", "name", "string", "utf8"]:
            if k in name_val and isinstance(name_val[k], str):
                s = name_val[k].strip()
                return s if s else None
        s = str(name_val).strip()
        return s if s else None
    s = str(name_val).strip()
    return s if s else None


def decode_us_message_frame(payload_bytes):
    return asn1_decode(payload_bytes, Asn1Type.US_MESSAGE_FRAME)


def detect_message_kind(raw_bytes, port_label):
    hx = raw_bytes.hex()
    if port_label in ("MAP", "SPAT", "SDSM"):
        return port_label
    for k, pref in HEX_PREFIX.items():
        if hx.startswith(pref):
            return k
    return "UNKNOWN"


def latlon_raw_to_deg(lat_raw, lon_raw):
    return (lat_raw / 1e7, lon_raw / 1e7)


def enu_from_latlon_deg(lat0_deg, lon0_deg, lat_deg, lon_deg):
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    dlat = lat - lat0
    dlon = lon - lon0

    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(lat0)
    return east, north


def latlon_from_enu_deg(lat0_deg, lon0_deg, east_m, north_m):
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    lat = lat0 + north_m / EARTH_RADIUS_M
    lon = lon0 + east_m / (EARTH_RADIUS_M * math.cos(lat0))
    return math.degrees(lat), math.degrees(lon)


def osm_tile_from_latlon(lat_deg, lon_deg, zoom):
    lat_deg = max(min(lat_deg, 85.05112878), -85.05112878)
    lon_deg = max(min(lon_deg, 180.0), -180.0)

    n = 2 ** zoom
    lat_rad = math.radians(lat_deg)
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return max(0, min(n - 1, x)), max(0, min(n - 1, y))


def latlon_from_osm_tile(x, y, zoom):
    n = 2 ** zoom
    lon_deg = x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n)))
    return math.degrees(lat_rad), lon_deg


def compute_bbox_from_points(points):
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def expand_bbox(bbox, pad_m):
    if bbox is None:
        return None
    minx, miny, maxx, maxy = bbox
    return (minx - pad_m, miny - pad_m, maxx + pad_m, maxy + pad_m)


# -------------------------
# Extractors (copied)
# -------------------------
def extract_map_intersections(decoded):
    intersections = []
    _, inters_val = find_first_key_recursive(
        decoded, ["intersections", "IntersectionGeometryList", "intersectionGeometryList"]
    )
    if inters_val is None:
        return intersections

    if isinstance(inters_val, dict) and "intersections" in inters_val:
        inters_val = inters_val["intersections"]

    if not isinstance(inters_val, (list, tuple)):
        return intersections

    for it in inters_val:
        iid = None
        _, name_val = find_first_key_recursive(it, ["name", "intersectionName", "descriptiveName"])
        name = normalize_name(name_val)

        if isinstance(it, dict) and "id" in it:
            id_obj = it["id"]
            if isinstance(id_obj, dict) and isinstance(id_obj.get("id"), int):
                iid = id_obj["id"]
            elif isinstance(id_obj, int):
                iid = id_obj

        if iid is None:
            _, iid_val = find_first_key_recursive(it, ["intersectionId", "intersectionID", "id"])
            if isinstance(iid_val, int):
                iid = iid_val
            elif isinstance(iid_val, dict) and isinstance(iid_val.get("id"), int):
                iid = iid_val["id"]

        ref_lat_raw = None
        ref_lon_raw = None
        if isinstance(it, dict) and isinstance(it.get("refPoint"), dict):
            ref_lat_raw = it["refPoint"].get("lat")
            ref_lon_raw = it["refPoint"].get("long")

        lane_set = it.get("laneSet") if isinstance(it, dict) else None

        if iid is not None:
            intersections.append({
                "intersection_id": iid,
                "name": name,
                "ref_lat_raw": ref_lat_raw,
                "ref_lon_raw": ref_lon_raw,
                "laneSet": lane_set,
            })

    return intersections


def extract_spat_states(decoded):
    out = {"intersection_id": None, "states": []}

    _, inters_val = find_first_key_recursive(decoded, ["intersections"])
    if isinstance(inters_val, (list, tuple)) and len(inters_val) > 0:
        first = inters_val[0]

        iid = None
        if isinstance(first, dict) and "id" in first:
            id_obj = first["id"]
            if isinstance(id_obj, dict) and isinstance(id_obj.get("id"), int):
                iid = id_obj["id"]

        if iid is None:
            _, iid_val = find_first_key_recursive(first, ["intersectionId", "intersectionID", "id"])
            if isinstance(iid_val, int):
                iid = iid_val
            elif isinstance(iid_val, dict) and isinstance(iid_val.get("id"), int):
                iid = iid_val["id"]

        out["intersection_id"] = iid

        states_val = first.get("states") if isinstance(first, dict) else None
        if states_val is None:
            _, states_val = find_first_key_recursive(first, ["states"])

        if isinstance(states_val, (list, tuple)):
            for st in states_val:
                sg = None
                if isinstance(st, dict) and isinstance(st.get("signalGroup"), int):
                    sg = st["signalGroup"]
                if sg is None:
                    _, sg_val = find_first_key_recursive(st, ["signalGroup"])
                    if isinstance(sg_val, int):
                        sg = sg_val

                event = None
                min_end = None
                max_end = None

                matches = find_all_matches_recursive(
                    st, lambda x: isinstance(x, dict) and "eventState" in x
                )
                if matches:
                    event = matches[0].get("eventState")
                    timing = matches[0].get("timing")
                    if isinstance(timing, dict):
                        min_end = timing.get("minEndTime")
                        max_end = timing.get("maxEndTime")

                out["states"].append({
                    "signalGroup": sg,
                    "eventState": event,
                    "minEndTime": min_end,
                    "maxEndTime": max_end,
                })

    return out


def _class_from_sdsm_obj(base_dict):
    _, cls_val = find_first_key_recursive(
        base_dict, ["objType", "objectType", "type", "classification", "class"]
    )
    if cls_val is None:
        return "unknown"
    if isinstance(cls_val, str):
        s = cls_val.lower()
        if "vru" in s or "ped" in s:
            return "pedestrian"
        if "bicy" in s or "bike" in s:
            return "bicycle"
        if "veh" in s or "car" in s:
            return "car"
        return cls_val
    if isinstance(cls_val, int):
        return f"type_{cls_val}"
    return str(cls_val)


def extract_sdsm(decoded):
    out = {"ref_lat_raw": None, "ref_lon_raw": None, "objects": []}

    _, refpos = find_first_key_recursive(decoded, ["refPos"])
    if isinstance(refpos, dict):
        out["ref_lat_raw"] = refpos.get("lat")
        out["ref_lon_raw"] = refpos.get("long")

    _, objs_val = find_first_key_recursive(
        decoded, ["objects", "detectedObjects", "participants", "detections"]
    )
    if not isinstance(objs_val, (list, tuple)):
        return out

    for obj in objs_val:
        common = obj.get("detObjCommon") if isinstance(obj, dict) else None
        base = common if isinstance(common, dict) else (obj if isinstance(obj, dict) else {})

        oid = None
        ox = None
        oy = None

        _, oid_val = find_first_key_recursive(base, ["objectID", "objID", "id", "trackId", "trackID"])
        if isinstance(oid_val, int):
            oid = oid_val

        _, pos_val = find_first_key_recursive(base, ["pos"])
        if isinstance(pos_val, dict):
            x_val = pos_val.get("offsetX")
            y_val = pos_val.get("offsetY")
            if isinstance(x_val, int):
                ox = x_val
            if isinstance(y_val, int):
                oy = y_val

        if ox is None:
            _, x_val = find_first_key_recursive(base, ["offsetX", "localX", "x"])
            if isinstance(x_val, int):
                ox = x_val
        if oy is None:
            _, y_val = find_first_key_recursive(base, ["offsetY", "localY", "y"])
            if isinstance(y_val, int):
                oy = y_val

        cls = _class_from_sdsm_obj(base)

        out["objects"].append({
            "id": oid,
            "class": cls,
            "offsetX": ox,
            "offsetY": oy,
        })

    return out


def sdsm_ros_msg_to_decoded(msg):
    objects = []

    for obj in getattr(msg, "objects", []):
        pos = getattr(obj, "position", None)
        obj_type = int(getattr(obj, "object_type", 0))

        vehicle_type = int(getattr(obj, "OBJECT_TYPE_VEHICLE", 1))
        vru_type = int(getattr(obj, "OBJECT_TYPE_VRU", 2))
        if obj_type == vehicle_type:
            class_name = "vehicle"
        elif obj_type == vru_type:
            class_name = getattr(obj, "vru_basic_type", "") or "vru"
        else:
            class_name = "unknown"

        objects.append({
            "objectID": int(getattr(obj, "object_id", 0)),
            "objectType": class_name,
            "pos": {
                "offsetX": int(getattr(pos, "offset_x", 0)),
                "offsetY": int(getattr(pos, "offset_y", 0)),
            },
        })

    return {
        "refPos": {
            "lat": int(getattr(msg, "ref_lat", 0)),
            "long": int(getattr(msg, "ref_lon", 0)),
        },
        "objects": objects,
    }


def spat_ros_msg_to_decoded(msg):
    spat = getattr(msg, "spat", None)
    intersections = []

    for inter in getattr(spat, "intersections", []):
        states = []

        for state in getattr(inter, "states", []):
            events = []

            for event in getattr(state, "events", []):
                timing = getattr(event, "timing", None)
                events.append({
                    "eventState": int(getattr(event, "event_state", 0)),
                    "timing": {
                        "minEndTime": (
                            int(getattr(timing, "min_end_time", 0))
                            if getattr(timing, "has_min_end_time", False)
                            else None
                        ),
                        "maxEndTime": (
                            int(getattr(timing, "max_end_time", 0))
                            if getattr(timing, "has_max_end_time", False)
                            else None
                        ),
                    },
                })

            states.append({
                "signalGroup": int(getattr(state, "signal_group", 0)),
                "events": events,
            })

        intersections.append({
            "id": {"id": int(getattr(inter, "intersection_id", 0))},
            "states": states,
        })

    return {"intersections": intersections}


# -------------------------
# MAP geometry (copied)
# -------------------------
def build_lane_polylines_from_laneSet(laneSet):
    polylines = {}
    if not isinstance(laneSet, (list, tuple)):
        return polylines

    for lane in laneSet:
        if not isinstance(lane, dict):
            continue
        lane_id = lane.get("laneID")
        nodeList = lane.get("nodeList")
        if lane_id is None or not nodeList:
            continue

        nodes = None
        if isinstance(nodeList, tuple) and len(nodeList) == 2 and nodeList[0] == "nodes":
            nodes = nodeList[1]
        elif isinstance(nodeList, dict) and "nodes" in nodeList:
            nodes = nodeList["nodes"]

        if not isinstance(nodes, (list, tuple)) or len(nodes) == 0:
            continue

        pts = []
        cur_x = 0
        cur_y = 0

        for nd in nodes:
            if not isinstance(nd, dict):
                continue
            delta = nd.get("delta")
            if not (isinstance(delta, tuple) and len(delta) == 2):
                continue
            _, dval = delta
            if not isinstance(dval, dict):
                continue

            dx = dval.get("x")
            dy = dval.get("y")
            if not isinstance(dx, int) or not isinstance(dy, int):
                continue

            cur_x += dx
            cur_y += dy
            pts.append((cur_x * MAP_XY_UNIT_METERS, cur_y * MAP_XY_UNIT_METERS))

        if len(pts) >= 2:
            polylines[lane_id] = pts

    return polylines


def convert_map_polylines_to_global(ref_lat_deg, ref_lon_deg, lane_polylines_local):
    if ref_lat_deg is None or ref_lon_deg is None:
        return {}

    base_e, base_n = enu_from_latlon_deg(
        TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, ref_lat_deg, ref_lon_deg
    )

    out = {}
    for lane_id, pts in lane_polylines_local.items():
        gpts = []
        for x_local_m, y_local_m in pts:
            gpts.append((base_e + x_local_m, base_n + y_local_m))
        out[lane_id] = gpts
    return out


# -------------------------
# ROS bridge (replaces UDP listener)
# -------------------------
class RosBridge(Node):
    def __init__(self, msg_q, stop_evt):
        super().__init__("rsu_monitor_bridge")
        self.msg_q = msg_q
        self.stop_evt = stop_evt
        raw_topic_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        if SDSM is not None:
            self.sub_sdsm = self.create_subscription(
                SDSM,
                "v2i/sdsm/raw",
                lambda msg: self._cb_sdsm_msg(msg, "v2i/sdsm/raw"),
                raw_topic_qos,
            )
            self.get_logger().info("Subscribed to v2i/sdsm/raw as v2i_sdsm_msgs/msg/SDSM")
        else:
            self.sub_sdsm = self.create_subscription(
                UInt8MultiArray,
                "v2i/sdsm/raw",
                lambda msg: self._cb_uint8(msg, "SDSM", "v2i/sdsm/raw"),
                raw_topic_qos,
            )
            self.get_logger().warn(
                "v2i_sdsm_msgs is not available; subscribing to v2i/sdsm/raw as UInt8MultiArray"
            )

        self.sub_map = self.create_subscription(
            UInt8MultiArray,
            "v2i/map/raw",
            lambda msg: self._cb_uint8(msg, "MAP", "v2i/map/raw"),
            raw_topic_qos,
        )

        if SpatPacket is not None:
            self.sub_spat = self.create_subscription(
                SpatPacket,
                "v2i/spat/raw",
                lambda msg: self._cb_spat_msg(msg, "v2i/spat/raw"),
                raw_topic_qos,
            )
            self.get_logger().info("Subscribed to v2i/spat/raw as v2i_spat_msgs/msg/SpatPacket")
        else:
            self.sub_spat = self.create_subscription(
                UInt8MultiArray,
                "v2i/spat/raw",
                lambda msg: self._cb_uint8(msg, "SPAT", "v2i/spat/raw"),
                raw_topic_qos,
            )
            self.get_logger().warn(
                "v2i_spat_msgs is not available; subscribing to v2i/spat/raw as UInt8MultiArray"
            )

    def _queue_decoded(self, kind, topic, decoded):
        try:
            self.msg_q.put_nowait({
                "ts": now_iso(),
                "port": None,
                "sender": f"ros:{topic}",
                "kind": kind,
                "decoded": decoded,
            })
        except queue.Full:
            pass

    def _cb_uint8(self, msg, kind, topic):
        try:
            payload = bytes(msg.data)
        except Exception:
            return
        try:
            decoded = decode_us_message_frame(payload)
        except Exception:
            # ignore decode failures
            return
        self._queue_decoded(kind, topic, decoded)

    def _cb_sdsm_msg(self, msg, topic):
        try:
            decoded = sdsm_ros_msg_to_decoded(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert SDSM message: {exc}")
            return
        self._queue_decoded("SDSM", topic, decoded)

    def _cb_spat_msg(self, msg, topic):
        try:
            decoded = spat_ros_msg_to_decoded(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert SPaT message: {exc}")
            return
        self._queue_decoded("SPAT", topic, decoded)


# -------------------------
# VIEW and MainWindow (copied from original)
# -------------------------
# ... (VirtualMapView, MainWindow and helpers are identical to original file above)

# For brevity we import the remaining UI classes from the original test6.py logic by
# copying relevant code below. (To keep this file self-contained, the UI code
# is included verbatim.)

# --- VirtualMapView and MainWindow implementations ---
class VirtualMapView(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)

        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)

        self._static_items = []
        self._lane_items = []
        self._lane_label_items = []
        self._dot_items = []
        self._dot_label_items = []
        self._tile_items = {}
        self._pending_tiles = set()
        self._wanted_tiles = set()
        self.osm_enabled = OSM_ENABLED_DEFAULT
        self.osm_cache_dir = OSM_CACHE_DIR
        self.osm_manager = QtNetwork.QNetworkAccessManager(self)
        self.osm_manager.finished.connect(self._on_osm_tile_reply)

        self._init_scene()

    def wheelEvent(self, event):
        zoom_in_factor = 1.15
        zoom_out_factor = 1.0 / zoom_in_factor
        if event.angleDelta().y() > 0:
            self.scale(zoom_in_factor, zoom_in_factor)
        else:
            self.scale(zoom_out_factor, zoom_out_factor)

    def fit_bbox_meters(self, bbox):
        if bbox is None:
            return
        self._ensure_osm_tiles_for_bbox(bbox)
        minx, miny, maxx, maxy = bbox
        rect = QtCore.QRectF(
            minx * PIXELS_PER_METER,
            -maxy * PIXELS_PER_METER,
            max((maxx - minx) * PIXELS_PER_METER, 100.0),
            max((maxy - miny) * PIXELS_PER_METER, 100.0),
        )
        self.fitInView(rect, QtCore.Qt.KeepAspectRatio)

    def set_osm_enabled(self, enabled):
        self.osm_enabled = bool(enabled)
        if self.osm_enabled:
            self._ensure_osm_tiles_for_bbox(self._current_bbox_meters())
        else:
            for item in self._tile_items.values():
                self.scene.removeItem(item)
            self._tile_items.clear()
            self._wanted_tiles.clear()

    def _current_bbox_meters(self):
        rect = self.scene.sceneRect()
        return (
            rect.left() / PIXELS_PER_METER,
            -rect.bottom() / PIXELS_PER_METER,
            rect.right() / PIXELS_PER_METER,
            -rect.top() / PIXELS_PER_METER,
        )

    def _tile_cache_path(self, key):
        z, x, y = key
        return os.path.join(self.osm_cache_dir, str(z), str(x), f"{y}.png")

    def _tile_scene_rect(self, key):
        z, x, y = key
        north_lat, west_lon = latlon_from_osm_tile(x, y, z)
        south_lat, east_lon = latlon_from_osm_tile(x + 1, y + 1, z)

        west_m, north_m = enu_from_latlon_deg(
            TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, north_lat, west_lon
        )
        east_m, south_m = enu_from_latlon_deg(
            TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, south_lat, east_lon
        )

        left = west_m * PIXELS_PER_METER
        top = -north_m * PIXELS_PER_METER
        width = (east_m - west_m) * PIXELS_PER_METER
        height = (north_m - south_m) * PIXELS_PER_METER
        return QtCore.QRectF(left, top, width, height)

    def _tile_keys_for_bbox_at_zoom(self, bbox, zoom):
        minx, miny, maxx, maxy = bbox
        corners = [
            latlon_from_enu_deg(TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, minx, miny),
            latlon_from_enu_deg(TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, minx, maxy),
            latlon_from_enu_deg(TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, maxx, miny),
            latlon_from_enu_deg(TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, maxx, maxy),
        ]

        tile_coords = [osm_tile_from_latlon(lat, lon, zoom) for lat, lon in corners]
        min_tx = min(x for x, _ in tile_coords)
        max_tx = max(x for x, _ in tile_coords)
        min_ty = min(y for _, y in tile_coords)
        max_ty = max(y for _, y in tile_coords)

        return [
            (zoom, x, y)
            for x in range(min_tx, max_tx + 1)
            for y in range(min_ty, max_ty + 1)
        ]

    def _tile_keys_for_bbox(self, bbox):
        for zoom in range(OSM_TILE_ZOOM, OSM_MIN_TILE_ZOOM - 1, -1):
            keys = self._tile_keys_for_bbox_at_zoom(bbox, zoom)
            if len(keys) <= OSM_MAX_TILES:
                return keys

        keys = self._tile_keys_for_bbox_at_zoom(bbox, OSM_MIN_TILE_ZOOM)
        if len(keys) <= OSM_MAX_TILES:
            return keys

        minx, miny, maxx, maxy = bbox
        center_lat, center_lon = latlon_from_enu_deg(
            TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, (minx + maxx) * 0.5, (miny + maxy) * 0.5
        )
        center_x, center_y = osm_tile_from_latlon(center_lat, center_lon, OSM_MIN_TILE_ZOOM)
        keys.sort(key=lambda key: (key[1] - center_x) ** 2 + (key[2] - center_y) ** 2)
        return keys[:OSM_MAX_TILES]

    def _add_osm_tile_item(self, key):
        if not self.osm_enabled or key in self._tile_items:
            return

        path = self._tile_cache_path(key)
        if not os.path.exists(path):
            return

        pixmap = QtGui.QPixmap(path)
        if pixmap.isNull():
            return

        rect = self._tile_scene_rect(key)
        item = self.scene.addPixmap(pixmap)
        item.setTransformationMode(QtCore.Qt.SmoothTransformation)
        item.setZValue(OSM_TILE_Z_VALUE)
        item.setPos(rect.left(), rect.top())
        item.setTransform(
            QtGui.QTransform().scale(
                rect.width() / pixmap.width(),
                rect.height() / pixmap.height(),
            )
        )
        self._tile_items[key] = item

    def _request_osm_tile(self, key):
        if key in self._pending_tiles:
            return

        self._pending_tiles.add(key)
        z, x, y = key
        url = OSM_TILE_URL_TEMPLATE.format(z=z, x=x, y=y)
        request = QtNetwork.QNetworkRequest(QtCore.QUrl(url))
        request.setRawHeader(b"User-Agent", OSM_USER_AGENT.encode("ascii"))
        reply = self.osm_manager.get(request)
        reply.setProperty("tile_key", f"{z}/{x}/{y}")

    def _on_osm_tile_reply(self, reply):
        key_text = reply.property("tile_key")
        try:
            z_text, x_text, y_text = str(key_text).split("/")
            key = (int(z_text), int(x_text), int(y_text))
        except (TypeError, ValueError):
            reply.deleteLater()
            return

        self._pending_tiles.discard(key)

        if reply.error() == QtNetwork.QNetworkReply.NoError:
            data = bytes(reply.readAll())
            if data:
                path = self._tile_cache_path(key)
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "wb") as f:
                    f.write(data)

                if key in self._wanted_tiles:
                    self._add_osm_tile_item(key)

        reply.deleteLater()

    def _ensure_osm_tiles_for_bbox(self, bbox):
        if not self.osm_enabled or bbox is None:
            return

        keys = set(self._tile_keys_for_bbox(bbox))
        self._wanted_tiles = keys

        for key, item in list(self._tile_items.items()):
            if key not in keys:
                self.scene.removeItem(item)
                del self._tile_items[key]

        for key in sorted(keys):
            if key in self._tile_items:
                continue

            if os.path.exists(self._tile_cache_path(key)):
                self._add_osm_tile_item(key)
            else:
                self._request_osm_tile(key)

    def _clear_items(self, items):
        for it in items:
            self.scene.removeItem(it)
        items.clear()

    def _init_scene(self):
        self.scene.clear()
        self._static_items.clear()
        self._lane_items.clear()
        self._lane_label_items.clear()
        self._dot_items.clear()
        self._dot_label_items.clear()
        self._tile_items.clear()
        self._wanted_tiles.clear()

        half_px = MAX_SCENE_HALF_SIZE_M * PIXELS_PER_METER
        self.scene.setSceneRect(-half_px, -half_px, 2 * half_px, 2 * half_px)
        self._ensure_osm_tiles_for_bbox(
            (
                -MAX_SCENE_HALF_SIZE_M,
                -MAX_SCENE_HALF_SIZE_M,
                MAX_SCENE_HALF_SIZE_M,
                MAX_SCENE_HALF_SIZE_M,
            )
        )

        axis_pen = QtGui.QPen(QtGui.QColor(200, 200, 200))
        axis_pen.setStyle(QtCore.Qt.DashLine)
        self._static_items.append(self.scene.addLine(-half_px, 0, half_px, 0, axis_pen))
        self._static_items.append(self.scene.addLine(0, -half_px, 0, half_px, axis_pen))

        t = self.scene.addText(f"Fixed origin = {TARGET_ORIGIN_NAME} (0,0)")
        t.setDefaultTextColor(QtGui.QColor(120, 120, 120))
        t.setPos(10, 10)
        self._static_items.append(t)

        self._draw_north_east_markers()
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _draw_arrow(self, x0, y0, x1, y1, color, label):
        pen = QtGui.QPen(color)
        pen.setWidth(2)
        self._static_items.append(self.scene.addLine(x0, y0, x1, y1, pen))

        dx = x1 - x0
        dy = y1 - y0
        ang = math.atan2(dy, dx)
        head_len = 14
        head_ang = math.radians(28)

        xh1 = x1 - head_len * math.cos(ang - head_ang)
        yh1 = y1 - head_len * math.sin(ang - head_ang)
        xh2 = x1 - head_len * math.cos(ang + head_ang)
        yh2 = y1 - head_len * math.sin(ang + head_ang)

        self._static_items.append(self.scene.addLine(x1, y1, xh1, yh1, pen))
        self._static_items.append(self.scene.addLine(x1, y1, xh2, yh2, pen))

        txt = self.scene.addText(label)
        txt.setDefaultTextColor(color)
        txt.setPos(x1 + 6, y1 + 6)
        self._static_items.append(txt)

    def _draw_north_east_markers(self):
        origin_x, origin_y = 0, 0
        L = 90
        self._draw_arrow(origin_x, origin_y, origin_x + L, origin_y, QtGui.QColor(0, 120, 255), "E")
        self._draw_arrow(origin_x, origin_y, origin_x, origin_y - L, QtGui.QColor(255, 80, 0), "N")

    def _color_for_class(self, cls):
        c = (cls or "unknown").lower()
        if "ped" in c:
            return QtGui.QColor(0, 200, 255)
        if "bicy" in c or "bike" in c:
            return QtGui.QColor(255, 200, 0)
        if "car" in c or "veh" in c:
            return QtGui.QColor(0, 220, 0)
        return QtGui.QColor(255, 0, 255)

    def draw_world(self, all_lane_records, all_object_records):
        self._clear_items(self._lane_items)
        self._clear_items(self._lane_label_items)
        self._clear_items(self._dot_items)
        self._clear_items(self._dot_label_items)

        minx = 1e9
        miny = 1e9
        maxx = -1e9
        maxy = -1e9

        lane_pen = QtGui.QPen(QtGui.QColor(30, 30, 30))
        lane_pen.setWidth(2)

        for rec in all_lane_records:
            pts = rec.get("points_global", [])
            lane_id = rec.get("lane_id")
            iid = rec.get("intersection_id")

            if not pts or len(pts) < 2:
                continue

            path = QtGui.QPainterPath()
            x0, y0 = pts[0]
            path.moveTo(x0 * PIXELS_PER_METER, -y0 * PIXELS_PER_METER)

            for xm, ym in pts[1:]:
                path.lineTo(xm * PIXELS_PER_METER, -ym * PIXELS_PER_METER)

            self._lane_items.append(self.scene.addPath(path, lane_pen))

            txt = self.scene.addText(f"{iid}:{lane_id}")
            txt.setDefaultTextColor(QtGui.QColor(80, 80, 80))
            txt.setPos(x0 * PIXELS_PER_METER + 5, -y0 * PIXELS_PER_METER + 5)
            self._lane_label_items.append(txt)

            for xm, ym in pts:
                minx = min(minx, xm)
                miny = min(miny, ym)
                maxx = max(maxx, xm)
                maxy = max(maxy, ym)

        for ob in all_object_records:
            xm = ob.get("x_m")
            ym = ob.get("y_m")
            if not isinstance(xm, (int, float)) or not isinstance(ym, (int, float)):
                continue

            px = xm * PIXELS_PER_METER
            py = -ym * PIXELS_PER_METER

            color = self._color_for_class(ob.get("class"))
            brush = QtGui.QBrush(color)
            pen = QtGui.QPen(QtCore.Qt.NoPen)

            self._dot_items.append(
                self.scene.addEllipse(
                    px - DOT_RADIUS_PX, py - DOT_RADIUS_PX,
                    2 * DOT_RADIUS_PX, 2 * DOT_RADIUS_PX,
                    pen, brush
                )
            )

            label = ob.get("id")
            if label is not None:
                t = self.scene.addText(str(label))
                t.setDefaultTextColor(QtGui.QColor(60, 60, 60))
                t.setPos(px + 6, py + 6)
                self._dot_label_items.append(t)

            minx = min(minx, xm)
            miny = min(miny, ym)
            maxx = max(maxx, xm)
            maxy = max(maxy, ym)

        if minx < 1e8:
            pad = SCENE_PAD_M
            bbox = (minx - pad, miny - pad, maxx + pad, maxy + pad)
            rect = QtCore.QRectF(
                bbox[0] * PIXELS_PER_METER,
                -bbox[3] * PIXELS_PER_METER,
                max((maxx - minx + 2 * pad) * PIXELS_PER_METER, 200.0),
                max((maxy - miny + 2 * pad) * PIXELS_PER_METER, 200.0),
            )
            self.scene.setSceneRect(rect)
            self._ensure_osm_tiles_for_bbox(bbox)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, msg_q, stop_evt):
        super().__init__()
        self.msg_q = msg_q
        self.stop_evt = stop_evt

        self.setWindowTitle(f"RSU Monitor – fixed global frame centered at {TARGET_ORIGIN_NAME}")
        self.resize(1650, 920)

        self.map_store = {}
        self.spat_store = {}
        self.sdsm_store = {}

        self.ui_frozen = False
        self.did_auto_fit_objects = False

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)

        self.lbl_status = QtWidgets.QLabel(
            f"Fixed visualization origin: {TARGET_ORIGIN_NAME} "
            f"({TARGET_ORIGIN_LAT:.6f}, {TARGET_ORIGIN_LON:.6f})"
        )
        top.addWidget(self.lbl_status, stretch=2)

        top.addWidget(QtWidgets.QLabel("Intersection:"))
        self.cmb_intersection = QtWidgets.QComboBox()
        top.addWidget(self.cmb_intersection, stretch=1)

        self.btn_zoom_selected = QtWidgets.QPushButton("Zoom to Selected")
        self.btn_zoom_selected.clicked.connect(self.zoom_to_selected_intersection)
        top.addWidget(self.btn_zoom_selected)

        self.btn_show_all = QtWidgets.QPushButton("Show All")
        self.btn_show_all.clicked.connect(self.show_all)
        top.addWidget(self.btn_show_all)

        self.btn_reset = QtWidgets.QPushButton("Reset View")
        self.btn_reset.clicked.connect(self.reset_view)
        top.addWidget(self.btn_reset)

        self.chk_osm = QtWidgets.QCheckBox("OSM map")
        self.chk_osm.setChecked(OSM_ENABLED_DEFAULT)
        top.addWidget(self.chk_osm)

        mid = QtWidgets.QHBoxLayout()
        root.addLayout(mid, stretch=1)

        left_box = QtWidgets.QGroupBox("Global Overlay (all MAP lanes + all SDSM objects)")
        left_layout = QtWidgets.QVBoxLayout(left_box)
        self.view = VirtualMapView()
        self.chk_osm.stateChanged.connect(
            lambda state: self.view.set_osm_enabled(state == QtCore.Qt.Checked)
        )
        left_layout.addWidget(self.view)
        self.lbl_osm_attribution = QtWidgets.QLabel("Map tiles: OpenStreetMap contributors")
        self.lbl_osm_attribution.setAlignment(QtCore.Qt.AlignRight)
        left_layout.addWidget(self.lbl_osm_attribution)
        mid.addWidget(left_box, stretch=3)

        right = QtWidgets.QVBoxLayout()
        mid.addLayout(right, stretch=2)

        spat_box = QtWidgets.QGroupBox("SPaT (all intersections)")
        spat_layout = QtWidgets.QVBoxLayout(spat_box)
        self.tbl_spat = QtWidgets.QTableWidget(0, 5)
        self.tbl_spat.setHorizontalHeaderLabels(
            ["IntersectionID", "SignalGroup", "EventState", "MinEndTime", "MaxEndTime"]
        )
        self.tbl_spat.horizontalHeader().setStretchLastSection(True)
        self.tbl_spat.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        spat_layout.addWidget(self.tbl_spat)
        right.addWidget(spat_box, stretch=1)

        sdsm_box = QtWidgets.QGroupBox("SDSM Objects (all sources)")
        sdsm_layout = QtWidgets.QVBoxLayout(sdsm_box)
        self.tbl_sdsm = QtWidgets.QTableWidget(0, 6)
        self.tbl_sdsm.setHorizontalHeaderLabels(
            ["Sender", "ObjID", "Class", "offsetX", "offsetY", "refPos"]
        )
        self.tbl_sdsm.horizontalHeader().setStretchLastSection(True)
        self.tbl_sdsm.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        sdsm_layout.addWidget(self.tbl_sdsm)
        right.addWidget(sdsm_box, stretch=1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.drain_queue)
        self.timer.start(int(1000 / GUI_UPDATE_HZ))

    def closeEvent(self, event):
        self.stop_evt.set()
        super().closeEvent(event)

    def reset_view(self):
        self.view._init_scene()
        self.redraw_overlay_only()
        self.show_all()

    def _make_sdsm_key(self, sender, ref_lat_deg, ref_lon_deg):
        if ref_lat_deg is None or ref_lon_deg is None:
            return (sender, "None", "None")
        return (sender, f"{ref_lat_deg:.6f}", f"{ref_lon_deg:.6f}")

    def _collect_all_lane_records(self):
        records = []
        for iid, ms in self.map_store.items():
            gpolys = ms.get("lane_polylines_global", {})
            for lane_id, pts in gpolys.items():
                records.append({
                    "intersection_id": iid,
                    "lane_id": lane_id,
                    "points_global": pts,
                })
        return records

    def _collect_all_sdsm_object_records(self):
        out = []
        for _, sd in self.sdsm_store.items():
            ref_lat_deg = sd.get("ref_lat_deg")
            ref_lon_deg = sd.get("ref_lon_deg")
            sender = sd.get("sender")

            if ref_lat_deg is None or ref_lon_deg is None:
                continue

            base_e, base_n = enu_from_latlon_deg(
                TARGET_ORIGIN_LAT, TARGET_ORIGIN_LON, ref_lat_deg, ref_lon_deg
            )

            for ob in sd.get("objects", []):
                ox = ob.get("offsetX")
                oy = ob.get("offsetY")
                if not isinstance(ox, int) or not isinstance(oy, int):
                    continue

                ox_m = ox * SDSM_OFFSET_UNIT_METERS
                oy_m = oy * SDSM_OFFSET_UNIT_METERS

                x_m = base_e + oy_m
                y_m = base_n + ox_m

                out.append({
                    "sender": sender,
                    "id": ob.get("id"),
                    "class": ob.get("class"),
                    "x_m": x_m,
                    "y_m": y_m,
                    "offsetX": ox,
                    "offsetY": oy,
                    "ref_lat_deg": ref_lat_deg,
                    "ref_lon_deg": ref_lon_deg,
                })
        return out

    def _update_intersection_dropdown(self):
        current = self.cmb_intersection.currentData()
        self.cmb_intersection.blockSignals(True)
        self.cmb_intersection.clear()
        self.cmb_intersection.addItem("All Intersections", None)

        items = []
        for iid, ms in self.map_store.items():
            name = ms.get("name")
            label = f"{name} ({iid})" if name else f"ID {iid}"
            items.append((label, iid))

        items.sort(key=lambda x: x[0])
        for label, iid in items:
            self.cmb_intersection.addItem(label, iid)

        if current is None:
            self.cmb_intersection.setCurrentIndex(0)
        else:
            idx = self.cmb_intersection.findData(current)
            if idx >= 0:
                self.cmb_intersection.setCurrentIndex(idx)

        self.cmb_intersection.blockSignals(False)

    def _update_spat_table(self):
        self.tbl_spat.setRowCount(0)
        rows = []
        for iid, rec in self.spat_store.items():
            for st in rec.get("states", []):
                rows.append((
                    iid,
                    st.get("signalGroup"),
                    st.get("eventState"),
                    st.get("minEndTime"),
                    st.get("maxEndTime"),
                ))

        rows.sort(key=lambda r: (str(r[0]), str(r[1])))

        for r, row in enumerate(rows):
            self.tbl_spat.insertRow(r)
            for c, val in enumerate(row):
                self.tbl_spat.setItem(r, c, QtWidgets.QTableWidgetItem(str(val)))

    def _update_sdsm_table(self):
        self.tbl_sdsm.setRowCount(0)
        rows = []
        for _, sd in self.sdsm_store.items():
            sender = sd.get("sender")
            ref_lat_deg = sd.get("ref_lat_deg")
            ref_lon_deg = sd.get("ref_lon_deg")
            ref_txt = (
                f"{ref_lat_deg:.6f}, {ref_lon_deg:.6f}"
                if ref_lat_deg is not None and ref_lon_deg is not None
                else "None"
            )
            for ob in sd.get("objects", []):
                rows.append((
                    sender,
                    ob.get("id"),
                    ob.get("class"),
                    ob.get("offsetX"),
                    ob.get("offsetY"),
                    ref_txt,
                ))

        rows.sort(key=lambda r: (str(r[0]), str(r[1])))

        for r, row in enumerate(rows):
            self.tbl_sdsm.insertRow(r)
            for c, val in enumerate(row):
                self.tbl_sdsm.setItem(r, c, QtWidgets.QTableWidgetItem(str(val)))

    def _intersection_bbox(self, iid):
        ms = self.map_store.get(iid)
        if not ms:
            return None

        pts = []
        for _, poly in ms.get("lane_polylines_global", {}).items():
            pts.extend(poly)

        bbox = compute_bbox_from_points(pts)
        return expand_bbox(bbox, INTERSECTION_ZOOM_PAD_M)

    def zoom_to_selected_intersection(self):
        iid = self.cmb_intersection.currentData()
        if iid is None:
            self.show_all()
            return

        bbox = self._intersection_bbox(iid)
        if bbox is not None:
            self.view.fit_bbox_meters(bbox)

    def show_all(self):
        lane_records = self._collect_all_lane_records()
        object_records = self._collect_all_sdsm_object_records()

        pts = []
        for rec in lane_records:
            pts.extend(rec.get("points_global", []))
        for ob in object_records:
            xm = ob.get("x_m")
            ym = ob.get("y_m")
            if isinstance(xm, (int, float)) and isinstance(ym, (int, float)):
                pts.append((xm, ym))

        bbox = compute_bbox_from_points(pts)
        bbox = expand_bbox(bbox, SCENE_PAD_M) if bbox else (-50, -50, 50, 50)
        self.view.fit_bbox_meters(bbox)

    def redraw_overlay_only(self):
        lane_records = self._collect_all_lane_records()
        object_records = self._collect_all_sdsm_object_records()
        self.view.draw_world(lane_records, object_records)

    def initialize_ui_once(self):
        if self.ui_frozen:
            return

        self._update_intersection_dropdown()
        self._update_spat_table()
        self._update_sdsm_table()

        self.lbl_status.setText(
            f"Origin: {TARGET_ORIGIN_NAME} | "
            f"MAP intersections: {len(self.map_store)} | "
            f"SPaT intersections: {len(self.spat_store)} | "
            f"SDSM sources: {len(self.sdsm_store)} | "
            f"MAP lock: {'ON' if LOCK_MAP_AFTER_FIRST else 'OFF'} | "
            f"UI frozen: {'ON' if FREEZE_UI_AFTER_FIRST_MAP else 'OFF'} | "
            "SDSM transform: swapped X/Y"
        )

        if FREEZE_UI_AFTER_FIRST_MAP:
            self.ui_frozen = True

    def refresh_all(self):
        self.redraw_overlay_only()

        if not self.ui_frozen:
            self._update_intersection_dropdown()
            self._update_spat_table()
            self._update_sdsm_table()
            self.lbl_status.setText(
                f"Origin: {TARGET_ORIGIN_NAME} | "
                f"MAP intersections: {len(self.map_store)} | "
                f"SPaT intersections: {len(self.spat_store)} | "
                f"SDSM sources: {len(self.sdsm_store)} | "
                f"Objects: {len(self._collect_all_sdsm_object_records())} | "
                f"MAP lock: {'ON' if LOCK_MAP_AFTER_FIRST else 'OFF'} | "
                f"UI frozen: {'ON' if FREEZE_UI_AFTER_FIRST_MAP else 'OFF'} | "
                "SDSM transform: swapped X/Y"
            )

    def drain_queue(self):
        drained = 0
        map_added = False
        overlay_updated = False
        objects_added = False

        while drained < 400:
            try:
                msg = self.msg_q.get_nowait()
            except queue.Empty:
                break
            drained += 1

            kind = msg.get("kind")
            decoded = msg.get("decoded")
            sender = msg.get("sender")
            ts = msg.get("ts")

            if kind == "MAP":
                inters = extract_map_intersections(decoded)

                for it in inters:
                    iid = it["intersection_id"]

                    if LOCK_MAP_AFTER_FIRST and iid in self.map_store:
                        continue

                    nm = it.get("name")
                    ref_lat_raw = it.get("ref_lat_raw")
                    ref_lon_raw = it.get("ref_lon_raw")

                    ref_lat_deg = None
                    ref_lon_deg = None
                    if isinstance(ref_lat_raw, int) and isinstance(ref_lon_raw, int):
                        ref_lat_deg, ref_lon_deg = latlon_raw_to_deg(ref_lat_raw, ref_lon_raw)

                    lane_polylines_local = build_lane_polylines_from_laneSet(it.get("laneSet"))
                    lane_polylines_global = convert_map_polylines_to_global(
                        ref_lat_deg, ref_lon_deg, lane_polylines_local
                    )

                    self.map_store[iid] = {
                        "name": nm,
                        "ref_lat_deg": ref_lat_deg,
                        "ref_lon_deg": ref_lon_deg,
                        "lane_polylines_local": lane_polylines_local,
                        "lane_polylines_global": lane_polylines_global,
                        "last_ts": ts,
                    }
                    map_added = True
                    overlay_updated = True

            elif kind == "SPAT":
                spat = extract_spat_states(decoded)
                iid = spat.get("intersection_id")
                if iid is None:
                    continue

                self.spat_store[iid] = {
                    "states": spat.get("states", []),
                    "last_ts": ts,
                }

                if not self.ui_frozen:
                    overlay_updated = True

            elif kind == "SDSM":
                sd = extract_sdsm(decoded)

                ref_lat_deg = None
                ref_lon_deg = None
                if isinstance(sd.get("ref_lat_raw"), int) and isinstance(sd.get("ref_lon_raw"), int):
                    ref_lat_deg, ref_lon_deg = latlon_raw_to_deg(sd["ref_lat_raw"], sd["ref_lon_raw"])

                key = self._make_sdsm_key(sender, ref_lat_deg, ref_lon_deg)
                self.sdsm_store[key] = {
                    "sender": sender,
                    "ref_lat_deg": ref_lat_deg,
                    "ref_lon_deg": ref_lon_deg,
                    "objects": sd.get("objects", []),
                    "last_ts": ts,
                }
                overlay_updated = True
                if sd.get("objects"):
                    objects_added = True

        if map_added:
            self.initialize_ui_once()

        if overlay_updated:
            if self.ui_frozen:
                self.redraw_overlay_only()
            else:
                self.refresh_all()

            if objects_added and not self.did_auto_fit_objects:
                self.show_all()
                self.did_auto_fit_objects = True


# -------------------------
# MAIN (ROS2-aware)
# -------------------------
def main():
    rclpy.init()

    msg_q = queue.Queue(maxsize=8000)
    stop_evt = threading.Event()

    bridge = RosBridge(msg_q, stop_evt)

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(msg_q, stop_evt)
    w.show()

    rc = app.exec_()

    # Shutdown
    stop_evt.set()
    try:
        bridge.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(rc)


if __name__ == "__main__":
    main()

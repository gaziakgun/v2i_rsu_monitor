v2i_rsu_monitor
================

GUI RSU monitor. The node subscribes to decoded V2I ROS messages on these topics:

- `v2i/sdsm/raw`  (SDSM messages)
- `v2i/map/raw`   (MAP messages)
- `v2i/spat/raw`  (SPaT messages)

The monitor supports two input formats:

- Custom decoded bag topics: `v2i_sdsm_msgs/msg/SDSM` on `/v2i/sdsm/raw`, `v2i_map_msgs/msg/MapData` on `/v2i/map/raw`, and `v2i_spat_msgs/msg/SpatPacket` on `/v2i/spat/raw`.
- Raw ASN.1 byte topics: `std_msgs/UInt8MultiArray` where `data` is a list of bytes (0-255). This fallback is used when the custom message packages are not available in the sourced ROS environment.

The node decodes ASN.1 using `pycmssdk` when byte topics are used and renders the GUI with PyQt5.

The GUI can also draw OpenStreetMap tiles behind the SDSM objects. Tiles are downloaded on demand and cached under `~/.cache/v2i_rsu_monitor/osm_tiles`, so the first run needs internet access for the map background.

Quickstart
----------

1. Prerequisites

- ROS 2 installed and sourced (Foxy, Humble, or later — ensure your distro matches installed Python packages).
- Custom V2I message packages from [`omerdurmus61/autoware_v2i_interfaces`](https://github.com/omerdurmus61/autoware_v2i_interfaces): `v2i_map_msgs`, `v2i_sdsm_msgs`, and `v2i_spat_msgs`.
- Python packages: `pycmssdk` and `PyQt5`.

Install the V2I message packages in a ROS 2 workspace before building this monitor:

```bash
cd ~/ros2_ws/src
git clone https://github.com/omerdurmus61/autoware_v2i_interfaces.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select v2i_map_msgs v2i_sdsm_msgs v2i_spat_msgs
source install/setup.bash
```

If you have the wheel in this repo, install it into your active Python environment:

```bash
pip install ./pycmssdk-20.64.1-py3-none-any.whl
pip install PyQt5
```

2. Build the package

From the workspace root (the folder containing this package), run:

```bash
# source your ROS2 install first, e.g. source /opt/ros/humble/setup.bash
# source the workspace that contains v2i_map_msgs/v2i_sdsm_msgs/v2i_spat_msgs
source ~/ros2_ws/install/setup.bash
colcon build --packages-select v2i_rsu_monitor
source install/setup.bash
```

3. Run the node

```bash
ros2 run v2i_rsu_monitor rsu_monitor
```

This opens the GUI and listens on the ROS topics instead of UDP.

Publishing test messages
------------------------

You can publish raw ASN.1 hex data as a `UInt8MultiArray`. Two quick ways:

- Using `ros2 topic pub` with explicit byte list (simple, for tiny examples):

```bash
# Example: publish bytes 0x00 0x12 0x... (replace with a real message)
ros2 topic pub --once /v2i/map/raw std_msgs/UInt8MultiArray "data: [0, 18, 255, 16]"
```

- Using a small Python helper to convert hex to bytes and publish (recommended for longer hex payloads):

```bash
python3 - <<'PY'
from std_msgs.msg import UInt8MultiArray
import rclpy

hexstr = '0012...'  # replace with real hex
b = bytes.fromhex(hexstr)
msg = UInt8MultiArray()
msg.data = list(b)

rclpy.init()
node = rclpy.create_node('pub_test')
pub = node.create_publisher(UInt8MultiArray, '/v2i/map/raw', 10)
# give rclpy a moment to register
import time
time.sleep(0.2)
pub.publish(msg)
node.destroy_node()
rclpy.shutdown()
PY
``



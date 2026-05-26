# juqiao_glove

ROS 2 driver and RViz visualization for the **Juqiao Industrial (矩侨工业)
fabric tactile glove** (models `JQGY-YL-11` left / `JQGY-YL-31` right).
Reverse-engineered from the official spec sheet (V1.1, 2026-03-23) and
verified against the real hardware.

The vendor ships no Linux driver and no ROS support. This package fills that
gap: it reads the standard USB-CDC serial stream (wired) or the wireless
USB-A receiver dongle (Bluetooth), parses the binary protocol, and publishes
clean ROS 2 topics — plus a heat-map visualization in RViz with the sensors
laid out anatomically on a hand silhouette.

Tested on **ROS 2 Humble** with Python 3.10, but it should work on any ROS 2
distro with `rclpy`, `sensor_msgs`, `std_msgs`, `visualization_msgs` and
`pyserial`.

## What this glove is

- 162 textile pressure sensors per glove, in a 16×16 = 256 logical grid (94
  slots are reserved padding).
- 1 bend (flex) sensor per finger — 5 extra channels for finger joint angles.
- 1× TDK-Invensense **ICM-42688** IMU per glove → 4× float32 unit quaternion
  embedded in every pressure frame.
- 100 Hz sample rate per glove (200 Hz aggregate USB throughput).
- Per-sensor max force: **350 N**, transmitted as uint8 (0–255).
- USB-CDC serial @ **921 600 baud** — appears as `/dev/ttyACM*` (wired) or
  `/dev/ttyUSB*` (wireless receiver) on Linux. No proprietary driver needed.

## Quick start

### 1. Build

```bash
cd <your_ws>/src
git clone https://github.com/chizelnut/JQ_glove_ROS2.git juqiao_glove
cd ..
colcon build --symlink-install --packages-select juqiao_glove
source install/setup.bash
```

### 2. Plug in one glove

The wired glove enumerates as `/dev/ttyACM0` (or `1`, etc.). The wireless
receiver dongle enumerates as `/dev/ttyUSB*`. Check with:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

**Note (Ubuntu):** the kernel's `brltty` Braille daemon hijacks one of the
two CH340 wireless receivers because their USB ID overlaps with a Baum
Braille device. To fix:

```bash
sudo systemctl mask brltty.service brltty-udev.service
sudo udevadm control --reload-rules
# unplug + replug the receivers
```

### 3. Launch the reader + RViz

```bash
# One glove (right hand on /dev/ttyACM0)
ros2 launch juqiao_glove single_glove_viz.launch.py \
    port:=/dev/ttyACM0 side:=right

# Both gloves at once
ros2 launch juqiao_glove dual_glove_viz.launch.py \
    left_port:=/dev/ttyACM1 right_port:=/dev/ttyACM0
```

You should see a hand-silhouette outline in RViz with 137 dots (162 sensors
total per the spec, of which 137 have explicit anatomical labels) lighting
green→yellow→red as you press the glove.

### 4. Headless mode (no RViz)

```bash
ros2 launch juqiao_glove single_glove.launch.py port:=/dev/ttyACM0 side:=right
ros2 topic hz /glove_right/forces   # should be ~100 Hz
```

## Topics

All topics live under the namespace `/glove_<side>/` (default `/glove_right/`).

| Topic      | Type                          | Contents |
|------------|-------------------------------|----------|
| `raw`      | `std_msgs/UInt8MultiArray`    | 256 raw ADC values, one per logical sensor slot. Unused slots stay 0. |
| `forces`   | `std_msgs/Float32MultiArray`  | Same 256 values converted to **Newtons** (× 350/255). |
| `imu`      | `sensor_msgs/Imu`             | Orientation quaternion from the on-glove IMU. Angular velocity / linear acceleration are not provided by the vendor protocol (their covariance is set to `-1` per REP-145). |
| `markers`  | `visualization_msgs/MarkerArray` | Hand silhouette + 137 sensor cubes for RViz. Only published when `viz_node` is running. |

## Sensor naming and access

The 256-slot array is exposed verbatim. To slice it by anatomical region
from Python:

```python
from juqiao_glove.layout import get_regions, to_zero_indexed
import rclpy
from std_msgs.msg import Float32MultiArray

regs = get_regions("right")
thumb_idx_0 = to_zero_indexed(regs["thumb_pressure"])   # 12-element list of 0-indexed offsets
index_bend_idx_0 = to_zero_indexed(regs["index_bend"])[0]

def on_forces(msg: Float32MultiArray):
    thumb_forces = [msg.data[i] for i in thumb_idx_0]      # 12 floats
    index_bend = msg.data[index_bend_idx_0]                 # 1 float
```

The full region list (per glove):

- `thumb_pressure`, `index_pressure`, `middle_pressure`, `ring_pressure`,
  `pinky_pressure` — 12 sensors each, row-major (`row 0 = tip`,
  `row 3 = base`; col indexing matches spec's "左→右" labelling).
- `thumb_bend`, `index_bend`, `middle_bend`, `ring_bend`, `pinky_bend` —
  1 flex sensor each (5 total).
- `palm` — 72 sensors in 5 rows (`row 0` adjacent to the fingers,
  `row 4` adjacent to the wrist).

That accounts for 137 of the spec's 162 sensors per glove. The remaining
25 are documented as "162 sensing points" in the vendor PDF but not
explicitly mapped in the per-region tables — likely on the back of the hand
or the cuff. They're still received in `raw` and `forces` and just won't
appear in any named slice or marker.

## Coordinate convention

Each glove publishes its own local TF frame, default
`glove_<side>_palm`:

- `+Y` : wrist → fingertips
- `+Z` : out of the palm (away from back-of-hand)
- `+X` : thumb side, *in the glove's own local frame*

The viz node mirrors `+X` for the left glove so both visuals look like a
palm-up view of their respective hand. Override with a launch arg or by
re-publishing the static TF.

## Wire protocol summary (verified)

The full protocol is documented in `juqiao_glove/layout.py` and
`juqiao_glove/protocol.py`. In one paragraph: the glove emits two
interleaved packet types over USB-CDC at 921 600 baud, each prefixed with
the sync sequence `AA 55 03 99` followed by a 1-byte packet-order
(`0x01` first, `0x02` second), a 1-byte sensor-type (`0x01`=Left Hand,
`0x02`=Right Hand, `0x03`=Left Foot, `0x04`=Right Foot, `0x05`=Whole
Body), then 128 sensor bytes. The second packet additionally appends a
16-byte (4× float32 LE, w-x-y-z) IMU quaternion. Concatenating both
packets gives the full 256-element pressure array. Both packet types
stream at 100 Hz independently. Glove side can be cross-verified from the
sensor-type byte even if a user mis-labels the device.

## Hardware acquisition

- Buy direct from Juqiao Industrial: <http://jq-industries.com>
- Vendor email: info@jq-industries.com / Phone: 010-62013231
- Vendor's spec sheet PDF is required to extend this driver to the foot
  / chest / full-body variants; the protocol is the same but the sensor
  index mappings differ.

## License

MIT — see [LICENSE](LICENSE).

## Acknowledgements

Built and tested in the context of an XHAND1 + Franka FR3 compliant-grasping
research project. The reverse-engineering work that pre-dated the vendor
spec sheet (cadence-locked frame discovery, byte-level pressure mapping,
glove-ID inference) is captured in the development history and is now
unnecessary — the spec sheet is authoritative — but the cadence-locked
parser remains useful for robust resync after USB hiccups.

This package is independent and not affiliated with or endorsed by Juqiao
Industrial.

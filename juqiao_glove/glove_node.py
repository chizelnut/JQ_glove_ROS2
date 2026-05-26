"""ROS 2 node: read one Juqiao glove serial stream, publish topics.

Topics (all under the node namespace, default `/glove_<side>`):
    raw     std_msgs/UInt8MultiArray   256 raw ADC values, indices 0..255
    forces  std_msgs/Float32MultiArray 256 values converted to Newtons
    imu     sensor_msgs/Imu            orientation = on-glove IMU quaternion

Parameters:
    port (str)           serial device path, default /dev/ttyACM0
    side (str)           'left' or 'right'; verified against the sensor-type
                         byte in the stream (warns on mismatch).
    frame_id (str)       TF frame for the IMU and (later) markers.
                         Default: 'glove_<side>_palm'.
    force_full_scale_n (float)  per-sensor max force; default 350.0 from spec.
    publish_raw / publish_forces / publish_imu (bool)  per-topic enables.
"""

from __future__ import annotations
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import UInt8MultiArray, Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Imu

try:
    import serial  # pyserial
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        "pyserial is required: pip install pyserial (or apt install python3-serial)"
    ) from e

from .protocol import GloveStreamParser, GloveSample
from .layout import (
    BAUD,
    SIDE_HEX,
    TOTAL_SENSOR_SLOTS,
    FORCE_FULL_SCALE_N,
)


def _make_uint8_msg(arr_bytes: bytes) -> UInt8MultiArray:
    msg = UInt8MultiArray()
    dim = MultiArrayDimension()
    dim.label = "sensor_index"
    dim.size = TOTAL_SENSOR_SLOTS
    dim.stride = TOTAL_SENSOR_SLOTS
    msg.layout.dim = [dim]
    msg.data = list(arr_bytes)
    return msg


def _make_float_msg(arr_bytes: bytes, scale: float) -> Float32MultiArray:
    msg = Float32MultiArray()
    dim = MultiArrayDimension()
    dim.label = "sensor_index"
    dim.size = TOTAL_SENSOR_SLOTS
    dim.stride = TOTAL_SENSOR_SLOTS
    msg.layout.dim = [dim]
    msg.data = [b * scale for b in arr_bytes]
    return msg


class GloveNode(Node):

    def __init__(self) -> None:
        super().__init__("juqiao_glove")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("side", "right")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("force_full_scale_n", FORCE_FULL_SCALE_N)
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_forces", True)
        self.declare_parameter("publish_imu", True)

        self._port = self.get_parameter("port").value
        self._side = str(self.get_parameter("side").value).lower()
        if self._side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {self._side!r}")
        self._expected_type = SIDE_HEX[self._side]
        fid = str(self.get_parameter("frame_id").value)
        self._frame_id = fid if fid else f"glove_{self._side}_palm"

        full_scale = float(self.get_parameter("force_full_scale_n").value)
        self._force_scale = full_scale / 255.0

        # Latched best-effort QoS for high-rate sensor data.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_raw = (
            self.create_publisher(UInt8MultiArray, "raw", qos)
            if self.get_parameter("publish_raw").value else None
        )
        self._pub_forces = (
            self.create_publisher(Float32MultiArray, "forces", qos)
            if self.get_parameter("publish_forces").value else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, "imu", qos)
            if self.get_parameter("publish_imu").value else None
        )

        self._sample_count = 0
        self._mismatch_warned = False

        self._parser = GloveStreamParser(on_sample=self._on_sample)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, name="glove-serial", daemon=True
        )

        # Open serial late so test code can construct the node without hardware.
        self._serial = serial.Serial(
            port=self._port,
            baudrate=BAUD,
            timeout=0.1,
            write_timeout=0.0,
        )
        self.get_logger().info(
            f"opened {self._port} @ {BAUD} as glove side='{self._side}' "
            f"(expecting sensor_type=0x{self._expected_type:02x}); "
            f"publishing on namespace {self.get_namespace()}"
        )
        self._thread.start()

        # Heartbeat: log sample rate every 5 s.
        self._last_heartbeat_count = 0
        self.create_timer(5.0, self._heartbeat)

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self._serial.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()

    # ---- serial reader -------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._serial.read(4096)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"serial read error: {e}")
                self._stop.wait(0.2)
                continue
            if chunk:
                self._parser.feed(chunk)

    # ---- sample handler ------------------------------------------------

    def _on_sample(self, sample: GloveSample) -> None:
        if sample.sensor_type != self._expected_type and not self._mismatch_warned:
            self.get_logger().warn(
                f"sensor_type mismatch: stream says 0x{sample.sensor_type:02x} "
                f"({sample.side}), node configured for side='{self._side}'. "
                f"Continuing to publish but topic namespace will be misleading."
            )
            self._mismatch_warned = True

        stamp = self.get_clock().now().to_msg()

        if self._pub_raw is not None:
            self._pub_raw.publish(_make_uint8_msg(sample.pressure))
        if self._pub_forces is not None:
            self._pub_forces.publish(_make_float_msg(sample.pressure, self._force_scale))
        if self._pub_imu is not None:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self._frame_id
            w, x, y, z = sample.quat_wxyz
            imu.orientation.w = float(w)
            imu.orientation.x = float(x)
            imu.orientation.y = float(y)
            imu.orientation.z = float(z)
            # Spec gives no covariance estimates; report unknown via small fixed values.
            # Per REP-145 conventions, leaving covariance[0] = -1 marks the field unknown.
            imu.orientation_covariance[0] = 0.0
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            self._pub_imu.publish(imu)

        self._sample_count += 1

    def _heartbeat(self) -> None:
        delta = self._sample_count - self._last_heartbeat_count
        rate = delta / 5.0
        self._last_heartbeat_count = self._sample_count
        self.get_logger().info(
            f"{self._sample_count} samples total, last 5 s = {rate:.1f} Hz "
            f"(spec target: 100 Hz)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GloveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

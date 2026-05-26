"""ROS 2 node: subscribe to /forces from one glove, publish RViz markers.

Markers (all in MarkerArray on topic `markers`):
    id 0           hand silhouette (LINE_STRIP) in semi-transparent gray
    id 1..N        one CUBE per mapped sensor, colored green->red by force
                   (uses sensor index + 100 as marker id to avoid clashes)

Frame: parameter `frame_id`, default `glove_<side>_palm`.

Coordinates from `layout.get_positions(side)` are in mm; multiplied by 1e-3
here for ROS metres.
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from .layout import (
    get_positions,
    HAND_SILHOUETTE_MM,
    FORCE_FULL_SCALE_N,
)

MM_TO_M = 1e-3

SENSOR_CUBE_SIZE_MM = 6.5          # marker edge in mm (matches ~6mm spec pad)
SENSOR_CUBE_HEIGHT_MM = 2.0        # thinner in Z so they look like pads

SILHOUETTE_LINE_WIDTH_M = 0.002    # 2 mm thick line


def _force_to_color(force_n: float, max_n: float) -> ColorRGBA:
    """Green (rest) -> yellow -> red (max). Linear interpolation in HSV would
    be smoother but linear RGB is fine for a single-channel heatmap."""
    t = max(0.0, min(1.0, force_n / max_n))
    c = ColorRGBA()
    if t < 0.5:
        # green -> yellow
        c.r = 2.0 * t
        c.g = 1.0
        c.b = 0.0
    else:
        # yellow -> red
        c.r = 1.0
        c.g = 2.0 * (1.0 - t)
        c.b = 0.0
    c.a = 1.0
    return c


class VizNode(Node):

    def __init__(self) -> None:
        super().__init__("juqiao_glove_viz")

        self.declare_parameter("side", "right")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("force_full_scale_n", FORCE_FULL_SCALE_N)
        self.declare_parameter("forces_topic", "forces")
        self.declare_parameter("markers_topic", "markers")
        self.declare_parameter("show_silhouette", True)
        self.declare_parameter("min_alpha", 0.35)  # alpha for resting sensors
        self.declare_parameter("max_alpha", 1.0)   # alpha for full-pressure sensors
        # Optional 3D mesh backdrop. Empty string = disabled. URI form:
        # 'package://juqiao_glove/meshes/hand_palm_up.stl' (Collada/STL only per RViz docs).
        self.declare_parameter("hand_mesh_uri", "")
        # Mesh transform: applied to bring the mesh into the glove_<side>_palm
        # frame. scale is per-axis (xyz). Translation is in metres; rpy in radians.
        self.declare_parameter("hand_mesh_scale_xyz", [0.0026, 0.0026, 0.0026])
        self.declare_parameter("hand_mesh_xyz", [0.0, 0.0, -0.005])
        self.declare_parameter("hand_mesh_rpy", [0.0, 0.0, 0.0])
        # Convenience: same rotation expressed in DEGREES. If any component is
        # non-zero, this takes precedence over hand_mesh_rpy (radians). Set to
        # [0, 0, 0] to use the radians parameter instead.
        self.declare_parameter("hand_mesh_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("hand_mesh_rgba", [0.85, 0.78, 0.72, 0.35])
        self.declare_parameter("hand_mesh_side_mirror", False)
        # When a mesh is loaded, rotate sensor markers + silhouette so they
        # align with the mesh's coordinate convention. The current mesh has
        # palm-normal = +Y and fingers = +Z, so we swap the Y and Z axes of
        # the sensor layout (which natively uses palm-normal = +Z, fingers = +Y).
        self.declare_parameter("align_to_mesh", True)

        self._side = str(self.get_parameter("side").value).lower()
        if self._side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {self._side!r}")
        fid = str(self.get_parameter("frame_id").value)
        self._frame_id = fid if fid else f"glove_{self._side}_palm"
        self._max_n = float(self.get_parameter("force_full_scale_n").value)
        self._show_sil = bool(self.get_parameter("show_silhouette").value)
        self._min_a = float(self.get_parameter("min_alpha").value)
        self._max_a = float(self.get_parameter("max_alpha").value)
        # Mesh URI is fixed at startup (file load happens once in RViz).
        # All other mesh transform params are read fresh on every publish so
        # `ros2 param set` from a terminal applies in real-time.
        self._mesh_uri = str(self.get_parameter("hand_mesh_uri").value).strip()
        self._mesh_side_mirror = bool(self.get_parameter("hand_mesh_side_mirror").value)
        self._align_to_mesh = bool(self.get_parameter("align_to_mesh").value) and bool(self._mesh_uri)

        self._positions = get_positions(self._side)  # {idx0: (mm,mm,mm)}
        self.get_logger().info(
            f"loaded {len(self._positions)} mapped sensor positions for {self._side} glove"
        )

        # Subscribe to /forces with BEST_EFFORT to match glove_node's sensor-data QoS.
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publish markers with RELIABLE so RViz (default RELIABLE) auto-connects.
        # Markers are ~60 Hz and only carry visualization data -- reliable is fine.
        pub_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("forces_topic").value),
            self._on_forces,
            sub_qos,
        )
        self._pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("markers_topic").value), pub_qos
        )

        # Pre-build the silhouette marker -- never changes.
        self._silhouette = self._build_silhouette()
        if self._mesh_uri:
            self.get_logger().info(
                f"hand mesh: {self._mesh_uri}  "
                f"(live-tunable via `ros2 param set {self.get_namespace()}/{self.get_name()} "
                f"hand_mesh_rpy '[r,p,y]'` etc.)"
            )

    # ------------------------------------------------------------------

    def _build_silhouette(self) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame_id
        m.ns = "glove_silhouette"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = SILHOUETTE_LINE_WIDTH_M
        m.color = ColorRGBA(r=0.55, g=0.55, b=0.6, a=0.7)
        m.pose.orientation.w = 1.0
        # HAND_SILHOUETTE_MM is drawn with thumb at +X. To match the spec
        # image convention (which is also the wearer-looking-at-own-palm
        # convention): LEFT hand palm-up has thumb on viewer's RIGHT (+X
        # in RViz), so no mirror; RIGHT hand palm-up has thumb on viewer's
        # LEFT, so mirror.
        x_sign = -1.0 if self._side == "right" else 1.0
        for x_mm, y_mm in HAND_SILHOUETTE_MM:
            p = Point()
            if self._align_to_mesh:
                # Mesh frame: palm-normal = +Y, fingers = +Z. Map silhouette's
                # (x, y) (x=lateral, y=wrist->fingers) to (x, 0, y).
                p.x = x_mm * MM_TO_M * x_sign
                p.y = 0.0
                p.z = y_mm * MM_TO_M
            else:
                p.x = x_mm * MM_TO_M * x_sign
                p.y = y_mm * MM_TO_M
                p.z = 0.0
            m.points.append(p)
        return m

    def _build_hand_mesh(self) -> Marker:
        from math import cos, sin
        m = Marker()
        m.header.frame_id = self._frame_id
        m.ns = "glove_hand_mesh"
        m.id = 1  # different from silhouette (0)
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = self._mesh_uri
        m.mesh_use_embedded_materials = False
        # Read mesh transform params LIVE each call so `ros2 param set` works.
        mesh_scale = list(self.get_parameter("hand_mesh_scale_xyz").value)
        mesh_xyz = list(self.get_parameter("hand_mesh_xyz").value)
        mesh_rpy_deg = list(self.get_parameter("hand_mesh_rpy_deg").value)
        if any(abs(v) > 1e-9 for v in mesh_rpy_deg):
            from math import pi
            mesh_rpy = [v * pi / 180.0 for v in mesh_rpy_deg]
        else:
            mesh_rpy = list(self.get_parameter("hand_mesh_rpy").value)
        mesh_rgba = list(self.get_parameter("hand_mesh_rgba").value)
        scale_x, scale_y, scale_z = mesh_scale
        x_sign = -1.0 if (self._side == "right" and self._mesh_side_mirror) else 1.0
        m.scale.x = scale_x * x_sign
        m.scale.y = scale_y
        m.scale.z = scale_z
        m.pose.position.x = mesh_xyz[0] * x_sign
        m.pose.position.y = mesh_xyz[1]
        m.pose.position.z = mesh_xyz[2]
        # Convert RPY -> quaternion (intrinsic XYZ: roll -> pitch -> yaw).
        roll, pitch, yaw = mesh_rpy
        cr, sr = cos(roll/2),  sin(roll/2)
        cp, sp = cos(pitch/2), sin(pitch/2)
        cyaw, syaw = cos(yaw/2), sin(yaw/2)
        m.pose.orientation.w = cr*cp*cyaw + sr*sp*syaw
        m.pose.orientation.x = sr*cp*cyaw - cr*sp*syaw
        m.pose.orientation.y = cr*sp*cyaw + sr*cp*syaw
        m.pose.orientation.z = cr*cp*syaw - sr*sp*cyaw
        r, g, b, a = mesh_rgba
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        return m

    def _build_sensor_marker(self, idx0: int, pos_mm, force_n: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame_id
        # Stamp set by caller (one stamp per MarkerArray for consistency).
        m.ns = "glove_sensors"
        m.id = idx0 + 100   # avoid colliding with silhouette id=0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        x_mm, y_mm, z_mm = pos_mm
        x_sign = -1.0 if self._side == "right" else 1.0
        m.pose.orientation.w = 1.0
        if self._align_to_mesh:
            # Mesh frame: palm-normal = +Y, fingers = +Z, thumb = +X.
            # Remap layout (x_mm = lateral, y_mm = wrist->fingers, z_mm = palm-out)
            # to (x_mesh = lateral, y_mesh = palm-out, z_mesh = wrist->fingers).
            m.pose.position.x = x_mm * MM_TO_M * x_sign
            m.pose.position.y = z_mm * MM_TO_M + 0.002  # palm-out offset
            m.pose.position.z = y_mm * MM_TO_M
            # Pad is thin along palm-normal axis (+Y in mesh frame).
            m.scale.x = SENSOR_CUBE_SIZE_MM * MM_TO_M
            m.scale.y = SENSOR_CUBE_HEIGHT_MM * MM_TO_M
            m.scale.z = SENSOR_CUBE_SIZE_MM * MM_TO_M
        else:
            m.pose.position.x = x_mm * MM_TO_M * x_sign
            m.pose.position.y = y_mm * MM_TO_M
            m.pose.position.z = z_mm * MM_TO_M + 0.002  # 2mm above silhouette plane
            m.scale.x = SENSOR_CUBE_SIZE_MM * MM_TO_M
            m.scale.y = SENSOR_CUBE_SIZE_MM * MM_TO_M
            m.scale.z = SENSOR_CUBE_HEIGHT_MM * MM_TO_M
        m.color = _force_to_color(force_n, self._max_n)
        # alpha scales with force so resting sensors fade back
        t = min(1.0, max(0.0, force_n / self._max_n))
        m.color.a = self._min_a + (self._max_a - self._min_a) * t
        return m

    # ------------------------------------------------------------------

    def _on_forces(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 256:
            self.get_logger().warn(
                f"forces msg has {len(msg.data)} elements, expected 256; skipping"
            )
            return
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # Stamp + emit silhouette
        if self._show_sil:
            self._silhouette.header.stamp = stamp
            arr.markers.append(self._silhouette)
        if self._mesh_uri:
            mesh = self._build_hand_mesh()
            mesh.header.stamp = stamp
            arr.markers.append(mesh)
        # One marker per mapped sensor
        for idx0, pos in self._positions.items():
            force = float(msg.data[idx0])
            m = self._build_sensor_marker(idx0, pos, force)
            m.header.stamp = stamp
            arr.markers.append(m)
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

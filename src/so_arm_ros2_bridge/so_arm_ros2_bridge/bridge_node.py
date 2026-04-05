from __future__ import annotations

import json
import math
import os
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from std_srvs.srv import SetBool
import yaml

from .robot_interface import RobotInterface, MockRobotInterface
from .unit_converter import (
    lerobot_to_ros,
    ros_to_lerobot,
    check_joint_limits,
    velocity_guard,
    pct_to_rad,
)


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_READ_RATE_HZ   = 50.0    # 50 Hz → 20 ms loop
DEFAULT_MAX_DELTA_RAD  = 0.05    # ~2.9° per step at 50 Hz → ~145°/s max
DEFAULT_MODE           = "ros_to_robot"
GRIPPER_JOINT_NAME     = "gripper"

# Topic names
JOINT_STATES_TOPIC    = "/joint_states"
JOINT_COMMANDS_TOPIC  = "/joint_commands"
STATUS_TOPIC          = "/bridge/status"

class SOArmBridgeNode(Node):
    """Bidirectional bridge between SO-ARM10x physical servos and ROS2."""

    def __init__(self):
        super().__init__("so_arm_bridge")

        # ── Declare parameters ───────────────────────────────────────────────
        self._declare_parameters()

        # ── Read parameters ──────────────────────────────────────────────────
        self._port              = self.get_parameter("port").value
        self._calibration_path  = self.get_parameter("calibration_path").value
        self._mode              = self.get_parameter("mode").value
        self._read_rate         = self.get_parameter("read_rate").value
        self._max_delta_rad     = self.get_parameter("max_delta_rad").value
        self._use_mock          = self.get_parameter("use_mock").value
        self._joint_config_path = self.get_parameter("joint_config_path").value

        # ── Load joint config (limits, names, gripper mapping) ───────────────
        self._joint_limits: Dict[str, Tuple[float, float]] = {}
        self._joint_names: List[str] = []
        self._load_joint_config()

        # ── State ────────────────────────────────────────────────────────────
        self._estop: bool = False
        self._write_enabled: bool = self._mode in ("ros_to_robot", "bidirectional")
        self._last_read: Dict[str, float] = {}          # radians
        self._last_command: Optional[JointState] = None
        self._command_timestamp: float = 0.0
        self._command_timeout_s: float = 2.0   # stop if no command for 2 s
        self._diag: Dict = {"commands_received": 0, "reads": 0, "write_errors": 0}

        # create a LeRobot "robot" instance
        self._robot = self._create_robot_interface()

        # set up publishers
        self._js_pub = self.create_publisher(JointState, JOINT_STATES_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # subscribe to /joint_commands
        self._cmd_sub = self.create_subscription(
            JointState, JOINT_COMMANDS_TOPIC, self._on_joint_command, 10
        )

        self._mode_srv = self.create_service(
            SetBool, "/bridge/set_mode", self._handle_set_mode
        )
        self._estop_srv = self.create_service(
            SetBool, "/bridge/estop", self._handle_estop
        )

        # ── Timers ───────────────────────────────────────────────────────────
        period = 1.0 / self._read_rate
        self._read_timer = self.create_timer(period, self._read_and_publish)

        # Slower status timer — 1 Hz
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"SO-ARM bridge started — mode={self._mode}, "
            f"port={self._port}, read_rate={self._read_rate} Hz, "
            f"mock={self._use_mock}"
        )

    # Read in parameters

    def _declare_parameters(self) -> None:
        desc = lambda d: ParameterDescriptor(description=d)
        self.declare_parameter("port", "/dev/ttyACM0",
            desc("Serial port for the Feetech bus servo adapter"))
        self.declare_parameter("calibration_path",
            str(Path.home() / ".cache/huggingface/lerobot/calibration/my_arm.json"),
            desc("Path to lerobot calibration JSON"))
        self.declare_parameter("mode", DEFAULT_MODE,
            desc("Operating mode: ros_to_robot | robot_to_ros | bidirectional"))
        self.declare_parameter("read_rate", DEFAULT_READ_RATE_HZ,
            desc("Servo polling rate (Hz)"))
        self.declare_parameter("max_delta_rad", DEFAULT_MAX_DELTA_RAD,
            desc("Max radians per step velocity guard"))
        self.declare_parameter("use_mock", False,
            desc("Use software mock (no hardware)"))
        self.declare_parameter("joint_config_path",
            str(Path(__file__).parent.parent / "config" / "joint_config.yaml"),
            desc("Path to joint_config.yaml"))

    # ── Initialisation helpers ───────────────────────────────────────────────

    def _load_joint_config(self) -> None:
        """Parse joint_config.yaml into limits and name list."""
        cfg_path = Path(self._joint_config_path)
        if not cfg_path.exists():
            self.get_logger().warn(
                f"joint_config.yaml not found at {cfg_path}. "
                "Using ±π limits for all joints."
            )
            # Provide sensible SO-101 defaults so the node still works
            self._joint_names = [
                "shoulder_pan_joint", "shoulder_lift_joint",
                "elbow_flex_joint", "wrist_flex_joint",
                "wrist_roll_joint", "gripper_joint",
            ]
            for name in self._joint_names:
                self._joint_limits[name] = (-math.pi, math.pi)
            return

        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        joints = cfg.get("joints", [])
        for j in joints:
            ros_name = j["ros_name"]
            lo = float(j.get("limit_lower_rad", -math.pi))
            hi = float(j.get("limit_upper_rad",  math.pi))
            self._joint_names.append(ros_name)
            self._joint_limits[ros_name] = (lo, hi)

        self.get_logger().info(
            f"Loaded {len(self._joint_names)} joints from {cfg_path}"
        )

    def _create_robot_interface(self):
        if self._use_mock:
            self.get_logger().warn("Using MOCK robot interface — no hardware communication.")
            iface = MockRobotInterface(motor_names=self._get_motor_names_from_config())
            iface.connect()
            return iface
        
        if not self._calibration_path or not Path(self._calibration_path).exists():
            raise FileNotFoundError(
                f"Calibration file not found: {self._calibration_path}\n"
                "Run lerobot-calibrate first, or set use_mock:=true for testing."
            )

        iface = RobotInterface(
            port=self._port,
            calibration_path=self._calibration_path,
            connect_on_init=True,
            torque_enabled=self._mode != "robot_to_ros"
        )
        return iface

    def _get_motor_names_from_config(self) -> List[str]:
        """Return lerobot motor names (not ROS joint names) from config."""
        cfg_path = Path(self._joint_config_path)
        if not cfg_path.exists():
            return ["shoulder_pan", "shoulder_lift", "elbow_flex",
                    "wrist_flex", "wrist_roll", "gripper"]
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        return [j["motor_name"] for j in cfg.get("joints", [])]

    # main loop: translate lerobot state data to ROS JointState msg

    def _read_and_publish(self) -> None:
        """Timer callback: read servo positions, publish JointState."""
        try:
            # Read raw positions from hardware (degrees / percent)
            pos_deg = self._robot.read_positions_deg()
            vel_deg = self._robot.read_velocities_deg_s()
        except Exception as exc:
            self.get_logger().error(f"Servo read failed: {exc}", throttle_duration_sec=5.0)
            self._diag["write_errors"] += 1
            return

        # Convert to radians using joint config mapping
        pos_rad = self._deg_map_to_rad(pos_deg)
        vel_rad = {name: math.radians(v) for name, v in vel_deg.items()}

        # Cache for velocity guard
        self._last_read = pos_rad

        # Build and publish JointState
        msg = self._build_joint_state(pos_rad, vel_rad)
        self._js_pub.publish(msg)
        self._diag["reads"] += 1

        # ── ROS → Robot: apply pending command ──────────────────────────────
        if self._write_enabled and not self._estop:
            self._apply_pending_command(pos_rad)

    def _build_joint_state(
        self,
        pos_rad: Dict[str, float],
        vel_rad: Dict[str, float],
    ) -> JointState:
        """Build a sensor_msgs/JointState from position/velocity dicts."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        for ros_name in self._joint_names:
            motor_name = self._ros_to_motor_name(ros_name)
            msg.name.append(ros_name)
            msg.position.append(pos_rad.get(motor_name, 0.0))
            msg.velocity.append(vel_rad.get(motor_name, 0.0))
            msg.effort.append(0.0)   # effort not available on STS3215

        return msg

    # ── Command subscription ─────────────────────────────────────────────────

    def _on_joint_command(self, msg: JointState) -> None:
        """
        Receive a JointState command from Gazebo/RViz.

        We store the message and apply it on the next read_timer tick so that
        the velocity guard can compare against the most recent actual position.
        """
        if self._estop:
            self.get_logger().warn(
                "E-STOP active — ignoring joint command.", throttle_duration_sec=2.0
            )
            return
        if not self._write_enabled:
            return

        self._last_command = msg
        self._command_timestamp = time.monotonic()
        self._diag["commands_received"] += 1

    def _apply_pending_command(self, current_pos_rad: Dict[str, float]) -> None:
        """Write the buffered command to the arm (called from read timer)."""
        if self._last_command is None:
            return

        # Command staleness check
        age = time.monotonic() - self._command_timestamp
        if age > self._command_timeout_s:
            self._last_command = None
            return

        cmd = self._last_command

        # Build target dict: {motor_name: target_rad}
        target_rad: Dict[str, float] = {}
        for ros_name, position in zip(cmd.name, cmd.position):
            motor_name = self._ros_to_motor_name(ros_name)
            if motor_name and not math.isnan(position) and not math.isinf(position):
                target_rad[motor_name] = position

        if not target_rad:
            return

        # Joint limit check
        ok, msg_text = check_joint_limits(target_rad, self._get_motor_limits())
        if not ok:
            self.get_logger().warn(
                f"Command rejected — {msg_text}", throttle_duration_sec=1.0
            )
            return

        # Velocity guard: clamp per-step delta
        motor_current = {
            self._ros_to_motor_name(rn): v
            for rn, v in current_pos_rad.items()
        }
        clamped, was_clamped = velocity_guard(
            motor_current, target_rad, self._max_delta_rad
        )
        if was_clamped:
            self.get_logger().debug(
                "Velocity guard clamped one or more joints.", throttle_duration_sec=0.5
            )

        # Convert to LeRobot units and write
        motor_limits = self._get_motor_limits()
        lerobot_vals = ros_to_lerobot(clamped, motor_limits, GRIPPER_JOINT_NAME)

        try:
            self._robot.write_positions_deg(lerobot_vals)
        except Exception as exc:
            self.get_logger().error(f"Servo write failed: {exc}", throttle_duration_sec=5.0)
            self._diag["write_errors"] += 1

    # ── Services ─────────────────────────────────────────────────────────────

    def _handle_set_mode(self, request: SetBool.Request, response: SetBool.Response):
        """SetBool: True = enable write path, False = read-only."""
        self._write_enabled = request.data
        state = "enabled" if self._write_enabled else "disabled"
        response.success = True
        response.message = f"Write path {state}."
        self.get_logger().info(f"Write path {state} via service call.")
        return response

    def _handle_estop(self, request: SetBool.Request, response: SetBool.Response):
        """SetBool: True = activate e-stop, False = clear."""
        self._estop = request.data
        if self._estop:
            self.get_logger().warn("*** EMERGENCY STOP ACTIVATED ***")
        else:
            self.get_logger().info("Emergency stop cleared.")
        response.success = True
        response.message = f"E-stop {'active' if self._estop else 'cleared'}."
        return response

    # ── Status publisher ─────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        status = {
            "mode": self._mode,
            "write_enabled": self._write_enabled,
            "estop": self._estop,
            "mock": self._use_mock,
            "reads": self._diag["reads"],
            "commands_received": self._diag["commands_received"],
            "write_errors": self._diag["write_errors"],
            "joints": self._joint_names,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Helpers: name mapping ─────────────────────────────────────────────────

    def _ros_to_motor_name(self, ros_name: str) -> Optional[str]:
        """
        Map a URDF joint name to a LeRobot motor name.

        The joint_config.yaml provides the authoritative mapping.
        Falls back to stripping '_joint' suffix for convenience.
        """
        cfg_path = Path(self._joint_config_path)
        if not hasattr(self, "_ros_to_motor_map"):
            self._ros_to_motor_map: Dict[str, str] = {}
            if cfg_path.exists():
                with open(cfg_path) as f:
                    cfg = yaml.safe_load(f)
                for j in cfg.get("joints", []):
                    self._ros_to_motor_map[j["ros_name"]] = j["motor_name"]

        if ros_name in self._ros_to_motor_map:
            return self._ros_to_motor_map[ros_name]
        # Fallback: strip '_joint' suffix
        return ros_name.replace("_joint", "")

    def _deg_map_to_rad(self, deg_map: Dict[str, float]) -> Dict[str, float]:
        """Convert {motor_name: degrees} → {motor_name: radians}."""
        result = {}
        motor_limits = self._get_motor_limits()
        for motor_name, val in deg_map.items():
            lo, hi = motor_limits.get(motor_name, (-math.pi, math.pi))
            if motor_name == GRIPPER_JOINT_NAME:
                result[motor_name] = pct_to_rad(val, lo, hi)
            else:
                result[motor_name] = math.radians(val)
        return result

    def _get_motor_limits(self) -> Dict[str, Tuple[float, float]]:
        """Return limits keyed by motor name (not ROS joint name)."""
        if not hasattr(self, "_motor_limits_cache"):
            self._motor_limits_cache: Dict[str, Tuple[float, float]] = {}
            cfg_path = Path(self._joint_config_path)
            if cfg_path.exists():
                with open(cfg_path) as f:
                    cfg = yaml.safe_load(f)
                for j in cfg.get("joints", []):
                    lo = float(j.get("limit_lower_rad", -math.pi))
                    hi = float(j.get("limit_upper_rad",  math.pi))
                    self._motor_limits_cache[j["motor_name"]] = (lo, hi)
        return self._motor_limits_cache

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Cleanly disconnect hardware before ROS node teardown."""
        self.get_logger().info("Shutting down bridge node…")
        try:
            self._robot.disconnect()
        except Exception as exc:
            self.get_logger().warn(f"Disconnect warning: {exc}")
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
# Entrypoint
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SOArmBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[FATAL] Bridge node crashed: {exc}")
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

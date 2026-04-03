from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Dict, Optional

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

logger = logging.getLogger(__name__)

# STS3215 hardware constants

TICKS_PER_REV      = 4096          # 12-bit encoder
DEGREES_PER_TICK   = 360.0 / TICKS_PER_REV   # 0.0879 °/tick
TICKS_PER_DEGREE   = TICKS_PER_REV / 360.0
DEFAULT_MOTOR_IDS  = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}

# P-gain tuned for SO-101 follower to reduce oscillation (default = 32)
DEFAULT_P_GAIN = 16
DEFAULT_I_GAIN = 0
DEFAULT_D_GAIN = 32

class RobotInterface:

    def __init__(
        self,
        port: str,
        calibration_path: str | Path,
        motor_ids: Optional[Dict[str, int]] = None,
        connect_on_init: bool = True,
        torque_enabled: bool = True,
    ):
        self.port = port
        self.calibration_path = Path(calibration_path)
        self.motor_ids = motor_ids or DEFAULT_MOTOR_IDS.copy()
        self._bus = None
        self._connected = False
        self._torque_enabled = torque_enabled

        if connect_on_init:
            self.connect()

    #Connect to the servo controller(FT-1,Waveshare board,etc.)
 
    def connect(self) -> None:
        """Open the serial bus and enable torque on all joints."""
        calibration = self._load_calibration()

        norm_body   = MotorNormMode.DEGREES       # joints 1-5 → [-180, 180]°
        norm_gripper = MotorNormMode.RANGE_0_100  # joint 6  → [0, 100]%

        motors = {}
        for name, motor_id in self.motor_ids.items():
            norm = norm_gripper if name == "gripper" else norm_body
            motors[name] = Motor(motor_id, "sts3215", norm)

        self._bus = FeetechMotorsBus(
            port=self.port,
            motors=motors,
            calibration=calibration,
        )
        self._bus.connect(handshake=True)
        if self._torque_enabled:
            # Follower arm — configure for active position control
            with self._bus.torque_disabled():
                self._bus.configure_motors()
                for motor_name in self._bus.motors:
                    self._bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
                    self._bus.write("P_Coefficient", motor_name, DEFAULT_P_GAIN)
                    self._bus.write("I_Coefficient", motor_name, DEFAULT_I_GAIN)
                    self._bus.write("D_Coefficient", motor_name, DEFAULT_D_GAIN)
        else:
            # Leader arm — leave torque off, arm stays compliant
            with self._bus.torque_disabled():
                self._bus.configure_motors()
            self._bus.disable_torque()     
        self._connected = True
        logger.info("RobotInterface connected on %s — %d motors", self.port, len(self.motor_ids))

    def disconnect(self) -> None:

        if self._bus is not None:
            try:
                self._bus.disconnect()
            except Exception as exc:
                logger.warning("Exception during disconnect: %s", exc)
        self._connected = False
        logger.info("RobotInterface disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def read_positions_deg(self) -> Dict[str, float]:
        """
        Return current joint positions as a dict of {joint_name: degrees}.

        Body joints (1-5): degrees in [-180, 180], zero at calibrated home.
        Gripper  (6):      percentage [0, 100].
        """
        self._check_connected()
        raw = self._bus.sync_read("Present_Position", normalize=True)
        return {name: float(val) for name, val in raw.items()}

    def read_positions_rad(self) -> Dict[str, float]:

        deg = self.read_positions_deg()
        return {name: math.radians(val) for name, val in deg.items()}

    def read_velocities_deg_s(self) -> Dict[str, float]:

        self._check_connected()
        try:
            raw = self._bus.sync_read("Present_Speed", normalize=False)
            # STS3215 speed register: signed, units ≈ 0.732 rpm per unit
            # Convert to deg/s:  unit × 0.732 rpm × 360°/rev / 60s
            return {name: float(v) * 0.732 * 360.0 / 60.0 for name, v in raw.items()}
        except Exception:
            # Velocity read is best-effort; return zeros if unsupported
            return {name: 0.0 for name in self.motor_ids}

    def write_positions_deg(self, positions: Dict[str, float]) -> None:
        """
        Send goal positions (degrees) to one or more joints.

        Parameters
        ----------
        positions : dict[str, float]
            Subset or full set of {joint_name: target_degrees}.
            Values outside [-180, 180] (or [0, 100] for gripper) are clamped
            by LeRobot's normalisation layer; hard limits are also enforced
            in the bridge node before this is called.
        """
        self._check_connected()
        self._bus.sync_write("Goal_Position", positions, normalize=True)

    def write_positions_rad(self, positions: Dict[str, float]) -> None:
        """
        Send goal positions (radians) to one or more body joints.
        Gripper should be provided in [0, 100] % — pass it as-is to
        write_positions_deg() after converting other joints.
        """
        deg_positions = {name: math.degrees(val) for name, val in positions.items()}
        self.write_positions_deg(deg_positions)

    # ── Calibration ──────────────────────────────────────────────────────────

    def _load_calibration(self):
        """Load calibration JSON as a dict[str, MotorCalibration]."""
        try:
            import draccus
            from lerobot.motors import MotorCalibration
        except ImportError:
            # draccus not installed; fall back to raw JSON load (newer lerobot
            # may not need draccus depending on install variant)
            logger.warning(
                "draccus not found; loading calibration as raw JSON dict. "
                "If calibration fails, install draccus: pip install draccus"
            )
            with open(self.calibration_path) as f:
                return json.load(f)

        with open(self.calibration_path) as f:
            try:
                from draccus import config_type
                with config_type("json"):
                    from lerobot.motors import MotorCalibration
                    calibration = draccus.load(dict[str, MotorCalibration], f)
            except Exception:
                f.seek(0)
                calibration = json.load(f)
        return calibration

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _check_connected(self) -> None:
        if not self._connected or self._bus is None:
            raise RuntimeError(
                "RobotInterface is not connected. Call connect() first."
            )

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()


# ──────────────────────────────────────────────────────────────────────────────
# Mock interface for unit testing (no hardware required)
# ──────────────────────────────────────────────────────────────────────────────

class MockRobotInterface:
    """
    Drop-in replacement for RobotInterface that simulates an arm in software.
    Used for unit tests and CI pipelines without physical hardware.

    Instantiate the bridge node with `use_mock=True` (see bridge_node.py).
    """

    def __init__(self, motor_names=None):
        self.motor_names = motor_names or list(DEFAULT_MOTOR_IDS.keys())
        self._positions: Dict[str, float] = {name: 0.0 for name in self.motor_names}
        self._connected = False

    def connect(self) -> None:
        self._connected = True
        logger.info("[MOCK] RobotInterface connected (simulation mode).")

    def disconnect(self) -> None:
        self._connected = False
        logger.info("[MOCK] RobotInterface disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def read_positions_deg(self) -> Dict[str, float]:
        return dict(self._positions)

    def read_positions_rad(self) -> Dict[str, float]:
        return {name: math.radians(v) for name, v in self._positions.items()}

    def read_velocities_deg_s(self) -> Dict[str, float]:
        return {name: 0.0 for name in self.motor_names}

    def write_positions_deg(self, positions: Dict[str, float]) -> None:
        for name, val in positions.items():
            if name in self._positions:
                self._positions[name] = val
        logger.debug("[MOCK] Wrote positions (deg): %s", positions)

    def write_positions_rad(self, positions: Dict[str, float]) -> None:
        self.write_positions_deg(
            {name: math.degrees(v) for name, v in positions.items()}
        )

    def __enter__(self):
        self.connect(); return self

    def __exit__(self, *_):
        self.disconnect()

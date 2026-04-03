import math
import time
import pytest

# ──────────────────────────────────────────────────────────────────────────────
# Unit converter tests (pure functions — no ROS2 required)
# ──────────────────────────────────────────────────────────────────────────────

from so_arm_ros2_bridge.unit_converter import (
    deg_to_rad, rad_to_deg,
    gripper_pct_to_rad, gripper_rad_to_pct,
    lerobot_to_ros, ros_to_lerobot,
    check_joint_limits, velocity_guard,
)

LIMITS = {
    "shoulder_pan":  (-math.pi / 2, math.pi / 2),
    "shoulder_lift": (-math.pi / 2, math.pi / 2),
    "elbow_flex":    (-math.pi / 2, math.pi / 2),
    "wrist_flex":    (-math.pi / 2, math.pi / 2),
    "wrist_roll":    (-math.pi,     math.pi),
    "gripper":       (0.0, 0.8),
}


class TestDegRadConversions:
    def test_zero(self):
        assert deg_to_rad(0.0) == pytest.approx(0.0)
        assert rad_to_deg(0.0) == pytest.approx(0.0)

    def test_ninety_degrees(self):
        assert deg_to_rad(90.0) == pytest.approx(math.pi / 2, rel=1e-6)

    def test_round_trip(self):
        for deg in [-180, -90, 0, 45, 90, 180]:
            assert rad_to_deg(deg_to_rad(float(deg))) == pytest.approx(float(deg), abs=1e-9)


class TestGripperConversion:
    def test_zero_pct(self):
        r = gripper_pct_to_rad(0.0, 0.0, 0.8)
        assert r == pytest.approx(0.0)

    def test_hundred_pct(self):
        r = gripper_pct_to_rad(100.0, 0.0, 0.8)
        assert r == pytest.approx(0.8)

    def test_fifty_pct(self):
        r = gripper_pct_to_rad(50.0, 0.0, 0.8)
        assert r == pytest.approx(0.4)

    def test_clamp_over(self):
        r = gripper_pct_to_rad(150.0, 0.0, 0.8)
        assert r == pytest.approx(0.8)

    def test_clamp_under(self):
        r = gripper_pct_to_rad(-10.0, 0.0, 0.8)
        assert r == pytest.approx(0.0)

    def test_round_trip(self):
        for pct in [0.0, 25.0, 50.0, 75.0, 100.0]:
            rad = gripper_pct_to_rad(pct, 0.0, 0.8)
            pct_back = gripper_rad_to_pct(rad, 0.0, 0.8)
            assert pct_back == pytest.approx(pct, abs=1e-6)


class TestBatchConversions:
    def test_lerobot_to_ros_body_joints(self):
        deg = {"shoulder_pan": 45.0, "elbow_flex": -30.0}
        rad = lerobot_to_ros(deg, LIMITS)
        assert rad["shoulder_pan"] == pytest.approx(math.radians(45.0), rel=1e-6)
        assert rad["elbow_flex"]   == pytest.approx(math.radians(-30.0), rel=1e-6)

    def test_lerobot_to_ros_gripper(self):
        deg = {"gripper": 50.0}
        rad = lerobot_to_ros(deg, LIMITS)
        assert rad["gripper"] == pytest.approx(0.4, rel=1e-6)

    def test_ros_to_lerobot_body_joints(self):
        rad = {"shoulder_pan": math.pi / 4}
        deg = ros_to_lerobot(rad, LIMITS)
        assert deg["shoulder_pan"] == pytest.approx(45.0, rel=1e-6)

    def test_ros_to_lerobot_clamped(self):
        # Value beyond limit should be clamped
        rad = {"shoulder_pan": math.pi}  # > π/2 limit
        deg = ros_to_lerobot(rad, LIMITS)
        assert deg["shoulder_pan"] == pytest.approx(90.0, rel=1e-4)

    def test_round_trip_body(self):
        original = {"shoulder_pan": math.pi / 4, "elbow_flex": -math.pi / 6}
        deg = ros_to_lerobot(original, LIMITS)
        back = lerobot_to_ros(deg, LIMITS)
        for name in original:
            assert back[name] == pytest.approx(original[name], rel=1e-5)


class TestJointLimits:
    def test_all_within(self):
        pos = {name: 0.0 for name in LIMITS}
        ok, msg = check_joint_limits(pos, LIMITS)
        assert ok
        assert msg == ""

    def test_one_violation(self):
        pos = {"shoulder_pan": math.pi}  # > π/2
        ok, msg = check_joint_limits(pos, LIMITS)
        assert not ok
        assert "shoulder_pan" in msg

    def test_tolerance(self):
        # Tiny overshoot within tolerance should pass
        pos = {"shoulder_pan": math.pi / 2 + 1e-4}
        ok, _ = check_joint_limits(pos, LIMITS, tolerance=1e-3)
        assert ok


class TestVelocityGuard:
    def test_no_clamping_needed(self):
        current = {"j1": 0.0, "j2": 0.0}
        target  = {"j1": 0.02, "j2": -0.01}
        clamped, was_clamped = velocity_guard(current, target, max_delta_rad=0.05)
        assert not was_clamped
        assert clamped["j1"] == pytest.approx(0.02)
        assert clamped["j2"] == pytest.approx(-0.01)

    def test_clamping_triggered(self):
        current = {"j1": 0.0}
        target  = {"j1": 1.0}   # too large
        clamped, was_clamped = velocity_guard(current, target, max_delta_rad=0.05)
        assert was_clamped
        assert clamped["j1"] == pytest.approx(0.05)

    def test_negative_clamping(self):
        current = {"j1": 0.5}
        target  = {"j1": -1.0}
        clamped, was_clamped = velocity_guard(current, target, max_delta_rad=0.05)
        assert was_clamped
        assert clamped["j1"] == pytest.approx(0.45)


# ──────────────────────────────────────────────────────────────────────────────
# MockRobotInterface tests (no ROS2, no hardware)
# ──────────────────────────────────────────────────────────────────────────────

from so_arm_ros2_bridge.robot_interface import MockRobotInterface


class TestMockInterface:
    def setup_method(self):
        self.robot = MockRobotInterface()
        self.robot.connect()

    def teardown_method(self):
        self.robot.disconnect()

    def test_initial_positions_zero(self):
        pos = self.robot.read_positions_deg()
        for v in pos.values():
            assert v == 0.0

    def test_write_then_read(self):
        self.robot.write_positions_deg({"shoulder_pan": 45.0, "gripper": 50.0})
        pos = self.robot.read_positions_deg()
        assert pos["shoulder_pan"] == pytest.approx(45.0)
        assert pos["gripper"]      == pytest.approx(50.0)

    def test_write_rad_then_read_rad(self):
        self.robot.write_positions_rad({"elbow_flex": math.pi / 4})
        pos_rad = self.robot.read_positions_rad()
        assert pos_rad["elbow_flex"] == pytest.approx(math.pi / 4, rel=1e-6)

    def test_context_manager(self):
        with MockRobotInterface() as robot:
            assert robot.is_connected
        assert not robot.is_connected

    def test_velocities_return_zero(self):
        vels = self.robot.read_velocities_deg_s()
        for v in vels.values():
            assert v == 0.0

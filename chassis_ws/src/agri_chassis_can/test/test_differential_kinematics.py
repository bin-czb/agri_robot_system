import math

from agri_chassis_can.differential_kinematics import DifferentialKinematics, integrate_midpoint


def make_kinematics():
    return DifferentialKinematics(
        wheel_radius_m=0.2,
        track_width_m=1.0,
        gear_ratio=1.0,
        driver_max_rpm=100.0,
    )


def test_straight_motion_is_symmetric():
    targets = make_kinematics().body_twist_to_targets(0.5, 0.0)
    assert math.isclose(targets.left_mps, targets.right_mps)
    assert targets.left_command == targets.right_command


def test_turn_has_opposite_wheel_bias():
    targets = make_kinematics().body_twist_to_targets(0.2, 0.2)
    assert targets.left_mps < targets.right_mps
    assert targets.left_command < targets.right_command


def test_pair_saturation_preserves_ratio():
    targets = make_kinematics().body_twist_to_targets(1.0, 0.5)
    assert max(abs(targets.left_command), abs(targets.right_command)) <= 10000
    command_ratio = targets.left_command / targets.right_command
    mps_ratio = targets.left_mps / targets.right_mps
    assert math.isclose(command_ratio, mps_ratio, rel_tol=2e-3, abs_tol=2e-3)


def test_midpoint_integrator_turns():
    x, y, yaw = integrate_midpoint(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
    assert x > 0.0
    assert y > 0.0
    assert math.isclose(yaw, 1.0)

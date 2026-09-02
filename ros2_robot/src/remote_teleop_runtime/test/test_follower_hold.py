from remote_teleop_runtime.follower import (
    FEEDBACK_CAN_FAULT_TIMEOUT_NS,
    FEEDBACK_CONTROL_TIMEOUT_NS,
    FollowerGateway,
    bounded_tracking_target,
)


def _gateway_with_feedback():
    gateway = object.__new__(FollowerGateway)
    gateway.enable_left = True
    gateway.have_right_feedback = True
    gateway.have_left_feedback = True
    gateway.positions = [float(index) for index in range(16)]
    gateway.hold_right = gateway.hold_left = None
    gateway.hold_right_gripper = gateway.hold_left_gripper = None
    return gateway


def test_hold_reference_is_a_snapshot_not_a_live_pose_alias():
    gateway = _gateway_with_feedback()

    gateway.capture_hold_reference()
    original_right = gateway.hold_right
    original_left = gateway.hold_left
    gateway.positions = [value + 1.0 for value in gateway.positions]

    assert gateway.hold_right == original_right
    assert gateway.hold_left == original_left
    assert gateway.hold_right == tuple(float(index) for index in range(8, 15))
    assert gateway.hold_left == tuple(float(index) for index in range(0, 7))


def test_clearing_run_reference_latches_the_current_safe_hold_pose():
    gateway = _gateway_with_feedback()
    gateway.run_leader_right = gateway.run_follower_right = (1.0,) * 7
    gateway.run_leader_left = gateway.run_follower_left = (1.0,) * 7
    gateway.run_leader_gripper = gateway.run_follower_gripper = 1.0
    gateway.run_leader_left_gripper = gateway.run_follower_left_gripper = 1.0
    gateway.last_target_right = gateway.last_target_left = (1.0,) * 7

    gateway.clear_run_reference()

    assert gateway.run_leader_right is None
    assert gateway.run_leader_left is None
    assert gateway.last_target_right is None
    assert gateway.last_target_left is None
    assert gateway.hold_right == tuple(float(index) for index in range(8, 15))
    assert gateway.hold_left == tuple(float(index) for index in range(0, 7))


def test_tracking_limit_is_relative_to_current_pose_not_startup_pose():
    # A joint can travel well beyond 0.20 rad in total. Only its instantaneous
    # command error is bounded, matching the pre-startup-hold behavior.
    assert bounded_tracking_target([0.60], [1.00]) == [0.80]
    assert bounded_tracking_target([0.79], [1.00]) == [0.99]
    assert bounded_tracking_target([0.99], [1.00]) == [1.00]


def test_tracking_target_preserves_absolute_one_to_one_joint_values():
    actual = [0.50, -0.40, 0.25]
    leader = [0.55, -0.45, 0.20]

    assert bounded_tracking_target(actual, leader) == leader


def test_feedback_control_pause_precedes_latched_can_fault():
    assert FEEDBACK_CONTROL_TIMEOUT_NS == 150_000_000
    assert FEEDBACK_CAN_FAULT_TIMEOUT_NS == 1_000_000_000
    assert FEEDBACK_CONTROL_TIMEOUT_NS < FEEDBACK_CAN_FAULT_TIMEOUT_NS

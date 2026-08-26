"""Build MoveGroup goals from a simple pose request (no ROS node logic)."""

from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

# arm key → (MoveIt group, ee link)
ARMS = {
    "left": ("left_arm", "openarm_left_hand"),
    "right": ("right_arm", "openarm_right_hand"),
}

_DEFAULT_VEL = 0.2
_DEFAULT_ACC = 0.2
_DEFAULT_FRAME = "world"
_POS_TOL = 0.001
_ORI_TOL = 0.1
_PLAN_ATTEMPTS = 10
_PLAN_TIME = 10.0


def resolve_arm(arm: str) -> tuple[str, str]:
    key = arm.strip().lower()
    if key not in ARMS:
        raise ValueError(f"arm must be one of {list(ARMS)}, got {arm!r}")
    return ARMS[key]


def build_move_group_goal(
    arm: str,
    pose: Pose,
    *,
    frame_id: str = "",
    vel_scale: float = 0.0,
    acc_scale: float = 0.0,
    plan_only: bool = False,
) -> MoveGroup.Goal:
    group_name, link_name = resolve_arm(arm)
    frame = frame_id.strip() or _DEFAULT_FRAME
    vel = vel_scale if vel_scale > 0.0 else _DEFAULT_VEL
    acc = acc_scale if acc_scale > 0.0 else _DEFAULT_ACC

    pos_c = PositionConstraint()
    pos_c.header.frame_id = frame
    pos_c.link_name = link_name
    pos_c.weight = 1.0
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [_POS_TOL]
    region = BoundingVolume()
    region.primitives = [sphere]
    region.primitive_poses = [Pose(position=pose.position, orientation=Quaternion(w=1.0))]
    pos_c.constraint_region = region

    ori_c = OrientationConstraint()
    ori_c.header.frame_id = frame
    ori_c.link_name = link_name
    ori_c.orientation = pose.orientation
    ori_c.absolute_x_axis_tolerance = _ORI_TOL
    ori_c.absolute_y_axis_tolerance = _ORI_TOL
    ori_c.absolute_z_axis_tolerance = _ORI_TOL
    ori_c.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [pos_c]
    constraints.orientation_constraints = [ori_c]

    request = MotionPlanRequest()
    request.group_name = group_name
    request.num_planning_attempts = _PLAN_ATTEMPTS
    request.allowed_planning_time = _PLAN_TIME
    request.max_velocity_scaling_factor = vel
    request.max_acceleration_scaling_factor = acc
    request.goal_constraints = [constraints]

    options = PlanningOptions()
    options.plan_only = plan_only

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = options
    return goal

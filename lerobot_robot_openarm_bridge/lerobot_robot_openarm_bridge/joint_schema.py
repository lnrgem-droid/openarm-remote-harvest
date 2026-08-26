ARM_DOF = 7
GRIPPER_OPEN_M = 0.044


def joint_names(side: str) -> list[str]:
    return [f"openarm_{side}_joint{i}" for i in range(1, ARM_DOF + 1)] + [
        f"openarm_{side}_finger_joint1"
    ]


LEFT_JOINTS = joint_names("left")
RIGHT_JOINTS = joint_names("right")
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


def values_to_action(left: list[float], right: list[float]) -> dict[str, float]:
    if len(left) != ARM_DOF + 1 or len(right) != ARM_DOF + 1:
        raise ValueError("OpenArm 双臂数据必须各包含 7 个关节和 1 个夹爪值")
    return {
        f"{name}.pos": float(value)
        for name, value in zip(ALL_JOINTS, [*left, *right], strict=True)
    }


def action_to_values(action: dict[str, float]) -> tuple[list[float], list[float]]:
    try:
        left = [float(action[f"{name}.pos"]) for name in LEFT_JOINTS]
        right = [float(action[f"{name}.pos"]) for name in RIGHT_JOINTS]
    except KeyError as exc:
        raise ValueError(f"action 缺少关节字段: {exc.args[0]}") from exc
    return left, right

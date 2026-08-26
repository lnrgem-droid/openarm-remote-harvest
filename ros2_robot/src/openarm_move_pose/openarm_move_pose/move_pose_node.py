#!/usr/bin/env python3
"""Action server /move_pose → MoveIt /move_action."""

import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes

from openarm_move_pose.action import MovePose
from openarm_move_pose.move_group_goal import build_move_group_goal


def _wait_future(future, timeout_sec: float) -> bool:
    """Block until future done; other MultiThreadedExecutor threads keep spinning."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.01)
    return True


class MovePoseNode(Node):
    def __init__(self):
        super().__init__("move_pose_node")
        # Separate groups so server execute can wait while client responses are handled.
        self._server_cg = ReentrantCallbackGroup()
        self._client_cg = ReentrantCallbackGroup()
        self._mg = ActionClient(
            self, MoveGroup, "move_action", callback_group=self._client_cg
        )
        self._server = ActionServer(
            self,
            MovePose,
            "move_pose",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._server_cg,
        )
        self.get_logger().info("move_pose_node ready → /move_pose")

    def _on_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _publish_status(self, goal_handle, status: str):
        fb = MovePose.Feedback()
        fb.status = status
        goal_handle.publish_feedback(fb)

    def _fail(self, goal_handle, result, message: str, code=None):
        result.success = False
        result.error_code = int(
            MoveItErrorCodes.FAILURE if code is None else code
        )
        result.message = message
        self.get_logger().error(message)
        goal_handle.abort()
        return result

    def _execute(self, goal_handle):
        req = goal_handle.request
        result = MovePose.Result()

        try:
            mg_goal = build_move_group_goal(
                req.arm,
                req.pose,
                frame_id=req.frame_id,
                vel_scale=req.vel_scale,
                acc_scale=req.acc_scale,
                plan_only=req.plan_only,
            )
        except ValueError as e:
            return self._fail(goal_handle, result, str(e))

        if not self._mg.wait_for_server(timeout_sec=5.0):
            return self._fail(
                goal_handle, result, "/move_action server not available"
            )

        self._publish_status(goal_handle, "sending")
        send_future = self._mg.send_goal_async(mg_goal)
        if not _wait_future(send_future, 30.0):
            return self._fail(
                goal_handle, result, "/move_action send_goal timed out"
            )

        mg_handle = send_future.result()
        if mg_handle is None:
            return self._fail(
                goal_handle, result, "/move_action returned no goal handle"
            )
        if not mg_handle.accepted:
            return self._fail(
                goal_handle,
                result,
                f"/move_action goal rejected (status={mg_handle.status})",
            )

        self._publish_status(
            goal_handle, "planning" if req.plan_only else "executing"
        )
        result_future = mg_handle.get_result_async()
        if not _wait_future(result_future, 300.0):
            return self._fail(
                goal_handle, result, "/move_action get_result timed out"
            )

        wrapped = result_future.result()
        if wrapped is None:
            return self._fail(
                goal_handle, result, "/move_action returned no result"
            )

        mg_result = wrapped.result
        code = int(mg_result.error_code.val)
        ok = code == MoveItErrorCodes.SUCCESS
        result.success = ok
        result.error_code = code
        result.message = "ok" if ok else f"MoveIt error_code={code}"

        if ok:
            self.get_logger().info(
                f"move_pose done arm={req.arm} code={code}"
            )
            goal_handle.succeed()
        else:
            self.get_logger().error(result.message)
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovePoseNode()
    # Need ≥2 threads: one in server execute (waiting), one for client I/O.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from std_srvs.srv import Empty

from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose

START_X = 5.544445   # turtlesim start position (same as reset service)
START_Y = 5.544445
MAX_SIDE = 4.5       # keeps the whole square inside the 0..11 window


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')
        cb = ReentrantCallbackGroup()
        self.reset_cli = self.create_client(Empty, 'reset_turtle', callback_group=cb)
        self.pose_client = ActionClient(self, GoToPose, 'go_to_pose', callback_group=cb)
        self.server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=cb)
        self.get_logger().info('draw_square action server ready')

    def goal_cb(self, goal_request):
        side = goal_request.side_length
        if side <= 0.0 or side > MAX_SIDE:
            self.get_logger().warn(f'Rejected side_length={side}: must be in (0, {MAX_SIDE}]')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def go_to(self, x, y, yaw):
        """Send ONE goal to go_to_pose and wait until it finishes."""
        goal = GoToPose.Goal()
        goal.target = TargetPose(x=float(x), y=float(y), yaw=float(yaw))
        self.get_logger().info(f'-> go_to_pose goal: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        handle = await self.pose_client.send_goal_async(goal)
        if not handle.accepted:
            return False
        res = await handle.get_result_async()
        return res.result.success

    async def execute_cb(self, goal_handle):
        side = goal_handle.request.side_length
        result = DrawSquare.Result()
        result.success = False

        if not self.reset_cli.wait_for_service(timeout_sec=2.0) \
                or not self.pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('reset_turtle or go_to_pose is not running')
            goal_handle.abort()
            return result

        # Always start from a clean screen and the known start pose
        await self.reset_cli.call_async(Empty.Request())

        x0, y0, L = START_X, START_Y, side
        # 4 corners; yaw = direction of the NEXT side, so each corner is 90 degrees
        corners = [
            (x0 + L, y0,     1.5708),
            (x0 + L, y0 + L, 3.1416),
            (x0,     y0 + L, -1.5708),
            (x0,     y0,     0.0),
        ]

        for i, (x, y, yaw) in enumerate(corners, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            fb = DrawSquare.Feedback()
            fb.current_side = i
            goal_handle.publish_feedback(fb)
            ok = await self.go_to(x, y, yaw)
            if not ok:
                self.get_logger().error(f'Side {i} failed')
                goal_handle.abort()
                return result

        result.success = True
        goal_handle.succeed()
        self.get_logger().info('Square done')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SquareServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


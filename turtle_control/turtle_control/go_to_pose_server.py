import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits


def normalize_angle(angle):
    """Wrap an angle into the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, limit):
    """Limit value to the range [-limit, limit]."""
    return max(-limit, min(limit, value))


class GoToPoseServer(Node):
    # Controller settings
    KP_LINEAR = 1.5       # gain: speed per unit of distance error
    KP_ANGULAR = 4.0      # gain: rotation speed per radian of angle error
    POS_TOLERANCE = 0.05  # metres (turtlesim units)
    YAW_TOLERANCE = 0.03  # radians
    TURN_FIRST = 0.5      # if heading error is bigger than this, turn without moving
    LOOP_PERIOD = 0.02    # seconds (50 Hz)
    TIMEOUT = 60.0        # seconds before a goal is aborted

    def __init__(self):
        super().__init__('go_to_pose_server')

        # One reentrant group so callbacks can run at the same time
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # Speed-limit service
        self.create_service(SetSpeedLimits, 'set_speed_limits',
                            self.set_limits_callback,
                            callback_group=self.cb_group)

        # Turtle communication
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10,
                                 callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Action server
        self.busy = False
        self.action_server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Ready. max_linear_speed={self.max_linear_speed}, '
            f'max_angular_speed={self.max_angular_speed}')

    # ---------- parameters ----------
    @property
    def max_linear_speed(self):
        return self.get_parameter('max_linear_speed').value

    @property
    def max_angular_speed(self):
        return self.get_parameter('max_angular_speed').value

    def on_parameter_change(self, params):
        for p in params:
            if p.name in ('max_linear_speed', 'max_angular_speed'):
                if not isinstance(p.value, (int, float)) or p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} must be greater than zero')
        return SetParametersResult(successful=True)

    def set_limits_callback(self, request, response):
        lin = float(request.max_linear_speed)
        ang = float(request.max_angular_speed)

        if lin <= 0.0 or ang <= 0.0:
            response.success = False
            response.message = 'Speeds must be greater than zero'
            return response

        results = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, lin),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, ang),
        ])
        response.success = all(r.successful for r in results)
        response.message = ('Speed limits updated' if response.success
                            else 'Could not update parameters')
        self.get_logger().info(response.message)
        return response

    # ---------- turtle ----------
    def pose_callback(self, msg):
        self.pose = msg

    def stop_turtle(self):
        self.cmd_pub.publish(Twist())  # all zeros

    # ---------- action callbacks ----------
    def goal_callback(self, goal_request):
        t = goal_request.target
        if not (0.0 <= t.x <= 11.0 and 0.0 <= t.y <= 11.0):
            self.get_logger().warn('Goal rejected: x and y must be between 0 and 11')
            return GoalResponse.REJECT
        if self.busy:
            self.get_logger().warn('Goal rejected: already driving to another goal')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Goal accepted: x={t.x:.2f}, y={t.y:.2f}, yaw={t.yaw:.2f}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.busy = True
        try:
            return self.drive(goal_handle)
        finally:
            self.stop_turtle()
            self.busy = False

    def drive(self, goal_handle):
        target = goal_handle.request.target
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()

        # Wait for the first pose message
        start = time.time()
        while self.pose is None and time.time() - start < 2.0:
            time.sleep(0.05)
        if self.pose is None:
            self.get_logger().error('No pose received. Is turtlesim running?')
            goal_handle.abort()
            result.success = False
            return result

        start = time.time()
        phase = 'move'
        step = 0

        while rclpy.ok():
            # Cancel?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_distance_error = self.distance_to(target)
                return result

            # Timeout?
            if time.time() - start > self.TIMEOUT:
                self.get_logger().error('Timeout: goal aborted')
                goal_handle.abort()
                result.success = False
                result.final_distance_error = self.distance_to(target)
                return result

            p = self.pose
            dx = target.x - p.x
            dy = target.y - p.y
            distance = math.hypot(dx, dy)
            cmd = Twist()

            if phase == 'move':
                if distance < self.POS_TOLERANCE:
                    phase = 'rotate'
                else:
                    heading_error = normalize_angle(math.atan2(dy, dx) - p.theta)
                    # Read the limits EVERY loop so changes take effect live
                    cmd.angular.z = clip(self.KP_ANGULAR * heading_error,
                                         self.max_angular_speed)
                    if abs(heading_error) < self.TURN_FIRST:
                        cmd.linear.x = min(self.KP_LINEAR * distance,
                                           self.max_linear_speed)

            if phase == 'rotate':
                yaw_error = normalize_angle(target.yaw - p.theta)
                if abs(yaw_error) < self.YAW_TOLERANCE:
                    break  # done
                cmd.angular.z = clip(self.KP_ANGULAR * yaw_error,
                                     self.max_angular_speed)

            self.cmd_pub.publish(cmd)

            # Feedback about 10 times per second
            step += 1
            if step % 5 == 0:
                feedback.distance_remaining = float(distance)
                goal_handle.publish_feedback(feedback)

            time.sleep(self.LOOP_PERIOD)

        self.stop_turtle()
        goal_handle.succeed()
        result.success = True
        result.final_distance_error = float(self.distance_to(target))
        self.get_logger().info('Goal reached')
        return result

    def distance_to(self, target):
        return math.hypot(target.x - self.pose.x, target.y - self.pose.y)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()


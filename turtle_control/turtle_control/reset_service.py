import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        cb = ReentrantCallbackGroup()
        self.clear_cli = self.create_client(Empty, '/clear', callback_group=cb)
        self.tele_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=cb)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen', callback_group=cb)
        self.srv = self.create_service(Empty, 'reset_turtle', self.reset_cb, callback_group=cb)
        self.get_logger().info('reset_turtle service ready')

    def set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = 179, 184, 255, 3  # turtlesim defaults
        req.off = 1 if off else 0
        return self.pen_cli.call_async(req)

    async def reset_cb(self, request, response):
        await self.set_pen(True)                      # pen up so no line is drawn
        tele = TeleportAbsolute.Request()
        tele.x, tele.y, tele.theta = 5.544445, 5.544445, 0.0
        await self.tele_cli.call_async(tele)          # back to start
        await self.clear_cli.call_async(Empty.Request())  # wipe the screen
        await self.set_pen(False)                     # pen down again
        return response


def main():
    rclpy.init()
    node = ResetService()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
    rclpy.shutdown()

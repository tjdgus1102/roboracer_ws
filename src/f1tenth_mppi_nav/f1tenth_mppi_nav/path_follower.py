import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class PathFollower(Node):
    """Loads a pre-recorded waypoint CSV (x,y,yaw in the map frame) and
    sends it directly to controller_server's follow_path action, so the
    MPPI controller tracks a fixed, offline-designed global path without
    needing an interactive 2D Goal Pose."""

    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('path_file', os.path.expanduser('~/roboracer_ws/paths/track1.csv'))
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('loop', False)

        self.path_file = self.get_parameter('path_file').value
        self.controller_id = self.get_parameter('controller_id').value
        self.loop = self.get_parameter('loop').value

        self._client = ActionClient(self, FollowPath, 'follow_path')
        self._path_msg = self._load_path()

        self.timer = self.create_timer(1.0, self.try_send_goal)
        self._sent = False

    def _load_path(self):
        path = Path()
        path.header.frame_id = 'map'
        with open(self.path_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                x, y, yaw = (float(v) for v in line.split(','))
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
        self.get_logger().info(f'Loaded {len(path.poses)} waypoints from {self.path_file}')
        return path

    def try_send_goal(self):
        if self._sent:
            return
        if not self._client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('follow_path action server not available yet')
            return
        self._sent = True
        self.timer.cancel()
        self.send_goal()

    def send_goal(self):
        goal = FollowPath.Goal()
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        goal.path = self._path_msg
        goal.controller_id = self.controller_id
        self.get_logger().info(f'Sending FollowPath goal with {len(goal.path.poses)} waypoints')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal rejected, will retry (controller_server may not be active yet)')
            self._sent = False
            self.timer.reset()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('FollowPath succeeded')
            if not self.loop:
                return
        else:
            self.get_logger().warn(
                f'FollowPath ended without success (status={status}), retrying shortly'
            )
        self._sent = False
        self.timer.reset()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

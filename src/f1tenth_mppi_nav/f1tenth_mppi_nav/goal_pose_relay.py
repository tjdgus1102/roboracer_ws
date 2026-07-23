import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseRelay(Node):
    """Bridges RViz2's built-in "2D Goal Pose" tool (which only publishes
    a PoseStamped on /goal_pose) into an actual NavigateToPose action
    goal for bt_navigator, since no Nav2-aware RViz panel is loaded."""

    def __init__(self):
        super().__init__('goal_pose_relay')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.get_logger().info('Waiting for navigate_to_pose action server...')

    def goal_cb(self, msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server not available yet')
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'Sending NavigateToPose goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )
        self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

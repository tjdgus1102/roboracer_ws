import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class CmdVelToAckermann(Node):
    """Converts Nav2's Twist output (linear.x, angular.z) into an
    AckermannDriveStamped command using the bicycle-model steering
    relation, since f1tenth_gym_ros expects speed + steering_angle
    rather than a body-frame twist."""

    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('min_speed_for_steering', 0.05)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('drive_topic', '/drive')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed_for_steering').value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def cmd_vel_cb(self, msg: Twist):
        speed = msg.linear.x
        steering_angle = 0.0
        if abs(speed) >= self.min_speed:
            steering_angle = math.atan2(self.wheelbase * msg.angular.z, speed)
            steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        # This car's motor is wired opposite to the VESC convention, so a
        # positive drive.speed command makes it reverse. Only the command
        # path is flipped: vesc_to_odom shares speed_to_erpm_gain, so
        # negating that parameter instead would also invert /odom, which
        # already reports forward motion as positive.
        out.drive.speed = -speed
        out.drive.steering_angle = steering_angle
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

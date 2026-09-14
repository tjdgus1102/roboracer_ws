import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_msgs.msg import SpeedLimit
import tf2_ros


class SpeedProfile(Node):
    """Feeds the offline TUM velocity profile to MPPI.

    nav_msgs/Path carries no velocity, and MPPI drives at a constant vx_max,
    so the per-point speeds from the raceline optimizer would otherwise be
    thrown away. Nav2's controller_server does expose a speed limit interface
    (nav2_msgs/SpeedLimit on /speed_limit) that rewrites the controller's
    vx_max at runtime, which is what this node drives: localize the car on the
    raceline, look up the profile speed there, publish it.

    Note this makes the profile a *ceiling*, not a target - MPPI still picks
    its own speed under the cap, and PathFollowCritic is what pushes it up
    against that cap on the straights.
    """

    def __init__(self):
        super().__init__('speed_profile')

        self.declare_parameter(
            'path_file',
            os.path.expanduser('~/roboracer_ws/raceline_opt/big_0723_raceline.csv'))
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lookahead_time', 0.3)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('rate', 50.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        self.lookahead_min = self.get_parameter('lookahead_min').value

        self.xy, self.vx = self._load(self.get_parameter('path_file').value)
        # The raceline is written open (start != goal by ~0.65 m) for the goal
        # checker's sake, but it is physically a loop, so the lookahead window
        # wraps. Spacing is uniform, so a distance window is a point count.
        self.spacing = float(np.median(np.linalg.norm(np.diff(self.xy, axis=0), axis=1)))

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def _load(self, path_file):
        data = np.loadtxt(path_file, comments='#', delimiter=',')
        if data.shape[1] < 4:
            raise RuntimeError(
                f'{path_file} has {data.shape[1]} columns, expected 4 (x,y,yaw,vx). '
                'Re-run raceline_opt/convert_and_plot.py to add the profile.')
        xy, vx = data[:, :2], data[:, 3]
        self.get_logger().info(
            f'Loaded {len(vx)} profile points from {path_file}: '
            f'{vx.min():.2f}-{vx.max():.2f} m/s')
        return xy, vx

    def tick(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'no map->{self.base_frame} yet: {e}', throttle_duration_sec=2.0)
            return

        pos = np.array([tf.transform.translation.x, tf.transform.translation.y])
        i = int(np.argmin(np.linalg.norm(self.xy - pos, axis=1)))

        # Brake on the *upcoming* speed, not the current one: at 20 Hz control
        # with a 2 s MPPI horizon plus ESC lag, applying the cap only once the
        # car is already at the corner entry is too late. The TUM profile has
        # its own braking ramp; this just gives it a head start.
        window = max(self.lookahead_min, self.lookahead_time * self.vx[i])
        n = max(1, int(round(window / self.spacing)))
        speed = float(self.vx[np.arange(i, i + n) % len(self.vx)].min())

        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.speed_limit = speed
        msg.percentage = False
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedProfile()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

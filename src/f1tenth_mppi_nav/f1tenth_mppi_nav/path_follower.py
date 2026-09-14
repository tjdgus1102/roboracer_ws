import math
import os
import statistics

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import tf2_ros


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
        # Lap forever by keeping the goal a fixed distance AHEAD of the car
        # instead of at a fixed place on the track. Neither alternative works on
        # a closed circuit: SimpleGoalChecker only measures distance to the
        # path's final pose and ignores how much of the path has been driven, so
        # re-sending the path on success ('loop') succeeds instantly with the
        # car parked on the goal, and baking N laps into one path succeeds on
        # lap 1 as the car drives over the goal's location.
        self.declare_parameter('continuous', False)
        # Path length sent ahead of the car. Must stay well under a lap so the
        # goal never lands near the car: on big_0723 (20.9 m) the closest a
        # 10 m-ahead goal ever comes is 4.9 m, against a 0.3 m tolerance.
        self.declare_parameter('lookahead', 10.0)
        # Must be under MPPIController's reset_period (1.0 s). setPlan() only
        # swaps the path -- it does NOT reset the optimizer (controller.cpp:123)
        # -- so re-sending this often preempts without losing the warm-started
        # control sequence. Going slower than reset_period would idle the
        # controller long enough to trigger a real reset between goals.
        self.declare_parameter('resend_period', 0.5)
        # The simulator names the car's frame 'ego_racecar/base_link';
        # the real car uses plain 'base_link'.
        self.declare_parameter('base_frame', 'base_link')

        self.path_file = self.get_parameter('path_file').value
        self.controller_id = self.get_parameter('controller_id').value
        self.loop = self.get_parameter('loop').value
        self.continuous = self.get_parameter('continuous').value
        self.base_frame = self.get_parameter('base_frame').value

        self._client = ActionClient(self, FollowPath, 'follow_path')

        if self.continuous:
            self._start_continuous()
        else:
            self._path_msg = self._load_path()
            self.timer = self.create_timer(1.0, self.try_send_goal)
            self._sent = False

    def _read_points(self):
        pts = []
        with open(self.path_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                # the raceline CSV carries a 4th vx column for speed_profile
                pts.append(tuple(float(v) for v in line.split(',')[:3]))
        return pts

    @staticmethod
    def _to_path(pts):
        path = Path()
        path.header.frame_id = 'map'
        for x, y, yaw in pts:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _load_path(self):
        pts = self._read_points()
        self.get_logger().info(f'Loaded {len(pts)} waypoints from {self.path_file}')
        return self._to_path(pts)

    def _start_continuous(self):
        pts = self._read_points()
        spacing = statistics.median(
            math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:]))
        # The CSV is written *open* -- the last point stops ~0.7 m short of the
        # first so a one-shot goal clears the start line. Indexing it modulo its
        # length would put that hole in the path once per lap, one 0.7 m step
        # where the spacing is otherwise 0.1 m, which the index-based critics
        # (PathFollowCritic.offset_from_furthest) would jump across. Close the
        # loop properly instead, then every wrap is seamless.
        self._loop_pts = pts + self._bridge(pts[-1], pts[0], spacing)
        self._n_ahead = max(2, int(round(
            self.get_parameter('lookahead').value / spacing)))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            f'Continuous lapping: {len(pts)} waypoints from {self.path_file} '
            f'closed to {len(self._loop_pts)} ({len(self._loop_pts) * spacing:.1f} m), '
            f'sending {self._n_ahead} points ahead every '
            f'{self.get_parameter("resend_period").value:.2f} s')
        self.timer = self.create_timer(
            self.get_parameter('resend_period').value, self.send_window)

    def send_window(self):
        if not self._client.server_is_ready():
            self.get_logger().warn(
                'follow_path action server not available yet', throttle_duration_sec=5.0)
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'no map->{self.base_frame} yet: {e}', throttle_duration_sec=2.0)
            return

        pos = (tf.transform.translation.x, tf.transform.translation.y)
        n = len(self._loop_pts)
        i = min(range(n), key=lambda k: math.dist(self._loop_pts[k][:2], pos))
        window = [self._loop_pts[(i + k) % n] for k in range(self._n_ahead)]

        goal = FollowPath.Goal()
        goal.path = self._to_path(window)
        goal.path.header.stamp = self.get_clock().now().to_msg()
        goal.controller_id = self.controller_id
        # Fire and forget: the next window preempts this one long before it is
        # driven out, so the result is never the interesting signal.
        self._client.send_goal_async(goal)

    @staticmethod
    def _bridge(a, b, spacing):
        """Points strictly between a and b at roughly `spacing`. Straight-line
        interpolation: over the 0.70 m start/finish gap the line turns 0.11 rad,
        so the chord departs from the true arc by ~1 cm -- a fifth of a costmap
        cell, and well inside what the raceline's own wall clearance absorbs.
        """
        n = int(round(math.dist(a[:2], b[:2]) / spacing)) - 1
        if n < 1:
            return []
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return [
            (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + dyaw * t)
            for t in ((i + 1) / (n + 1) for i in range(n))
        ]

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

import math
import os

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


class RecordPath(Node):
    """Records map->base_link tf while the car is driven manually
    (e.g. via teleop) and writes it out as a CSV waypoint path that
    path_follower.py can later replay through the MPPI controller."""

    def __init__(self):
        super().__init__('record_path')

        self.declare_parameter('output_file', os.path.expanduser('~/roboracer_ws/paths/track1.csv'))
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('record_hz', 5.0)
        self.declare_parameter('min_distance', 0.1)

        self.output_file = self.get_parameter('output_file').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        hz = self.get_parameter('record_hz').value
        self.min_distance = self.get_parameter('min_distance').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.timer = self.create_timer(1.0 / hz, self.tick)

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self.get_logger().info(
            f'Recording {self.map_frame} -> {self.base_frame} to {self.output_file}. '
            'Drive the car now; press Ctrl+C to stop and save.'
        )

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ExtrapolationException):
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

        if self.waypoints:
            lx, ly, _ = self.waypoints[-1]
            if math.hypot(x - lx, y - ly) < self.min_distance:
                return

        self.waypoints.append((x, y, yaw))
        self.get_logger().info(f'Recorded waypoint #{len(self.waypoints)}: x={x:.2f}, y={y:.2f}', throttle_duration_sec=1.0)

    def save(self):
        with open(self.output_file, 'w') as f:
            f.write('# x,y,yaw\n')
            for x, y, yaw in self.waypoints:
                f.write(f'{x:.4f},{y:.4f},{yaw:.4f}\n')
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = RecordPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import SpeedLimit
from tf2_ros import Buffer, TransformListener, TransformException

# Reference palette, first three categorical slots (light mode). Documented as
# all-pairs validated, so they stay distinguishable as overlapping lines.
ACTUAL, CAP, CMD = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK_2, SURFACE, GRID = '#0b0b0b', '#52514e', '#fcfcfb', '#dedcd6'
# Sequential blue, steps 150-650, for speed magnitude on the track map.
SEQ = ['#b7d3f6', '#86b6ef', '#3987e5', '#256abf', '#184f95', '#104281']


class RecordLap(Node):
    """Records a lap and plots what the car actually did against what the
    velocity profile asked for.

    Three traces answer three different questions, and you need all three to
    tell the failure modes apart: the published cap is what the profile
    demanded, cmd_vel is what MPPI chose to ask for under that cap, and odom is
    what the car managed. Cap high but cmd_vel low means MPPI is the limiter
    (critic tuning); cmd_vel high but odom low means the car is (grip, ESC,
    battery)."""

    def __init__(self):
        super().__init__('record_lap')

        self.declare_parameter(
            'path_file',
            os.path.expanduser('~/roboracer_ws/raceline_opt/big_0723_raceline.csv'))
        self.declare_parameter(
            'output_dir', os.path.expanduser('~/roboracer_ws/laps'))
        self.declare_parameter('map_yaml', os.path.expanduser('~/roboracer_ws/maps/big_0723.yaml'))
        # The simulator names the car's frame 'ego_racecar/base_link';
        # the real car uses plain 'base_link'.
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 50.0)

        self.path_file = self.get_parameter('path_file').value
        self.output_dir = self.get_parameter('output_dir').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.base_frame = self.get_parameter('base_frame').value

        raceline = np.loadtxt(self.path_file, comments='#', delimiter=',')
        self.ref_xy, self.ref_v = raceline[:, :2], raceline[:, 3]
        self.spacing = float(np.median(np.linalg.norm(np.diff(self.ref_xy, axis=0), axis=1)))

        self.v_odom = float('nan')
        self.v_cmd = float('nan')
        self.v_cap = float('nan')
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd, 10)
        self.create_subscription(SpeedLimit, 'speed_limit', self._cap, 10)

        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.rows = []
        self.t0 = None
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info('Recording. Drive one lap, then press Ctrl+C to save the plot.')

    def _odom(self, m):
        self.v_odom = m.twist.twist.linear.x

    def _cmd(self, m):
        self.v_cmd = m.linear.x

    def _cap(self, m):
        self.v_cap = m.speed_limit

    def tick(self):
        try:
            tf = self.buf.lookup_transform('map', self.base_frame, rclpy.time.Time())
        except TransformException:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.rows.append((now - self.t0,
                          tf.transform.translation.x, tf.transform.translation.y,
                          self.v_odom, self.v_cmd, self.v_cap))
        self.get_logger().info(
            f'{now - self.t0:5.1f}s  odom {self.v_odom:.2f}  cmd {self.v_cmd:.2f} '
            f' cap {self.v_cap:.2f} m/s', throttle_duration_sec=1.0)

    # --- output ------------------------------------------------------------
    def save(self):
        if len(self.rows) < 10:
            self.get_logger().warn(f'only {len(self.rows)} samples, nothing to plot')
            return
        os.makedirs(self.output_dir, exist_ok=True)
        d = np.array(self.rows)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        csv = os.path.join(self.output_dir, f'lap_{stamp}.csv')
        np.savetxt(csv, d, delimiter=',', fmt='%.4f',
                   header='t,x,y,v_odom,v_cmd,v_cap')
        self.get_logger().info(f'Saved {len(d)} samples to {csv}')
        self._plot(d, os.path.join(self.output_dir, f'lap_{stamp}.png'))

    def _project(self, xy):
        """Distance along the raceline, so the run lines up with the profile."""
        idx = np.array([np.argmin(np.linalg.norm(self.ref_xy - p, axis=1)) for p in xy])
        return idx * self.spacing, idx

    def _plot(self, d, png):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap

        # Recording starts before the car does, so drop the stationary head and
        # tail: otherwise the idle wait lands inside lap 1 and roughly doubles
        # the reported time, and the map gets a blob of dots at the start.
        mv = np.flatnonzero(d[:, 3] > 0.2)
        if len(mv):
            d = d[mv[0]:mv[-1] + 1].copy()
            d[:, 0] -= d[0, 0]

        t, xy = d[:, 0], d[:, 1:3]
        s, idx = self._project(xy)
        # A lap wrap would otherwise draw one long horizontal streak back across
        # the plot; break the line there instead.
        gap = np.where(np.abs(np.diff(s)) > self.spacing * len(self.ref_v) / 2)[0]
        s_plot = s.astype(float).copy()
        s_plot[gap] = np.nan

        # Split at start-line crossings and keep only the segments that actually
        # cover the track. Counting wraps alone would call a 1.4-lap recording
        # "2 laps"; a partial out-lap or a run that starts mid-track is common.
        track_len = self.spacing * len(self.ref_v)
        bounds = np.concatenate(([0], gap + 1, [len(s)]))
        lap_times, odom_ratio = [], []
        for a, b in zip(bounds[:-1], bounds[1:]):
            if s[a:b].ptp() <= 0.9 * track_len:
                continue
            lap_times.append(t[b - 1] - t[a])
            # Cross-check the y axis: v_odom is ERPM x speed_to_erpm_gain, so a
            # bad gain scales every number here. Integrating it over a lap must
            # come back to the track length that AMCL says was covered.
            odom_ratio.append(
                np.sum(d[a:b - 1, 3] * np.diff(t[a:b])) / s[a:b].ptp())
        moving = d[:, 3][d[:, 3] > 0.1]

        fig, (ax, ax2) = plt.subplots(
            2, 1, figsize=(11, 9), gridspec_kw={'height_ratios': [1, 1.25]})
        fig.patch.set_facecolor(SURFACE)

        for a in (ax, ax2):
            a.set_facecolor(SURFACE)
            for spine in ('top', 'right'):
                a.spines[spine].set_visible(False)
            for spine in ('left', 'bottom'):
                a.spines[spine].set_color(GRID)
            a.tick_params(colors=INK_2, labelsize=9)

        # --- speed along the raceline ---
        ref_s = np.arange(len(self.ref_v)) * self.spacing
        ax.plot(ref_s, self.ref_v, color=GRID, lw=6, solid_capstyle='round',
                zorder=1, label='profile (offline)')
        ax.plot(s_plot, d[:, 5], color=CAP, lw=2, zorder=3, label='published cap')
        ax.plot(s_plot, d[:, 4], color=CMD, lw=2, zorder=2, label='commanded (cmd_vel)')
        ax.plot(s_plot, d[:, 3], color=ACTUAL, lw=2.4, zorder=4, label='actual (odom)')

        ax.set_xlim(0, ref_s[-1])
        ax.set_ylim(0, max(3.2, np.nanmax(d[:, 3:6]) * 1.1))
        ax.grid(axis='y', color=GRID, lw=0.8, alpha=0.7)
        ax.set_axisbelow(True)
        ax.set_xlabel('distance along raceline [m]', color=INK_2, fontsize=10)
        ax.set_ylabel('speed [m/s]', color=INK_2, fontsize=10)
        leg = ax.legend(loc='lower right', frameon=False, fontsize=9, ncol=4)
        for txt in leg.get_texts():
            txt.set_color(INK_2)
        if lap_times:
            head = '  ·  '.join(f'lap {i + 1}: {v:.2f} s' for i, v in enumerate(lap_times))
        else:
            head = f'{t[-1]:.1f} s recorded (no complete lap)'
        if len(moving):
            head += f'   ·   mean {moving.mean():.2f} m/s, peak {moving.max():.2f} m/s'
        ax.set_title(head, color=INK, fontsize=13, loc='left', pad=12)
        if odom_ratio:
            r = float(np.mean(odom_ratio))
            note = f'odom vs AMCL distance: {r:.2f}x'
            if abs(r - 1.0) > 0.1:
                note += '  - speed_to_erpm_gain looks miscalibrated, the speed axis is off'
            # low-left is the empty corner: profile sits high, legend low-right
            ax.text(0.005, 0.03, note, transform=ax.transAxes, fontsize=9,
                    color=INK_2, va='bottom')
            self.get_logger().info(note)

        # --- where on the track it was slow ---
        try:
            import yaml
            from PIL import Image
            with open(self.map_yaml) as fh:
                m = yaml.safe_load(fh)
            grid = np.asarray(Image.open(
                os.path.join(os.path.dirname(self.map_yaml), m['image'])).convert('L'))
            res, (ox, oy) = float(m['resolution']), m['origin'][:2]
            ax2.imshow(grid, cmap='gray', origin='upper', vmin=0, vmax=255,
                       extent=[ox, ox + grid.shape[1] * res, oy, oy + grid.shape[0] * res])
        except (OSError, KeyError) as e:
            self.get_logger().warn(f'map not drawn: {e}')

        cmap = LinearSegmentedColormap.from_list('speed', SEQ)
        sc = ax2.scatter(xy[:, 0], xy[:, 1], c=d[:, 3], cmap=cmap, s=14, zorder=3)
        cb = fig.colorbar(sc, ax=ax2, label='actual speed [m/s]', shrink=0.85)
        cb.ax.yaxis.label.set_color(INK_2)
        cb.ax.tick_params(colors=INK_2, labelsize=9)
        cb.outline.set_visible(False)
        ax2.plot(self.ref_xy[:, 0], self.ref_xy[:, 1], color=GRID, lw=1.2, zorder=2)
        ax2.set_aspect('equal')
        # Crop to the track; the map covers a lot of unknown space around it.
        lo, hi = self.ref_xy.min(axis=0) - 1.0, self.ref_xy.max(axis=0) + 1.0
        ax2.set_xlim(lo[0], hi[0])
        ax2.set_ylim(lo[1], hi[1])
        ax2.set_xlabel('x [m]', color=INK_2, fontsize=10)
        ax2.set_ylabel('y [m]', color=INK_2, fontsize=10)
        ax2.set_title('driven line, coloured by measured speed',
                      color=INK, fontsize=11, loc='left', pad=8)

        fig.tight_layout()
        fig.savefig(png, dpi=130, facecolor=SURFACE, bbox_inches='tight')
        self.get_logger().info(f'Saved plot to {png}')


def main(args=None):
    rclpy.init(args=args)
    node = RecordLap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():  # ros2 run's own SIGINT handler may have shut down already
            rclpy.shutdown()


if __name__ == '__main__':
    main()

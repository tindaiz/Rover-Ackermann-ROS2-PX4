#!/usr/bin/env python3
"""
Trajectory Recorder Node
========================
Thu thập dữ liệu đồng thời từ 3 nguồn:
  1. /odom            → Vận tốc thực tế của xe
  2. /plan            → Quỹ đạo toàn cục do Nav2 Hybrid A* tính toán (Global Plan)
  3. /cmd_vel         → Vận tốc mong muốn từ MPPI Controller
  4. /amcl_pose       → Vị trí ước lượng từ AMCL (Localized Pose) dùng làm quỹ đạo thực tế.

Khi nhấn Ctrl+C, node sẽ tự động:
  - Lưu dữ liệu ra file CSV trong thư mục ~/rover_ackermann/results/
  - Vẽ và lưu các biểu đồ phân tích ra file PNG
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

import math
import csv
import os
from datetime import datetime
from rclpy.duration import Duration

# ─── Matplotlib ───────────────────────────────────────────────────────────────
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


class TrajectoryRecorder(Node):
    """Node thu thập và vẽ biểu đồ quỹ đạo bám waypoint."""

    def __init__(self):
        super().__init__('trajectory_recorder')

        qos_plan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self._plan_cb, qos_plan)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        
        self.declare_parameter('use_amcl', False)
        self.use_amcl = self.get_parameter('use_amcl').value

        if self.use_amcl:
            self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self._timer_cb)

        # ── Dữ liệu thu thập ─────────────────────────────────────────────────
        # Quỹ đạo thực tế 
        self.actual_x = []
        self.actual_y = []
        self.actual_yaw = []
        self.actual_time = []

        # Quỹ đạo toàn cục
        self.plan_x = []
        self.plan_y = []

        # Vận tốc (Lưu timestamp riêng biệt để nội suy)
        self.cmd_time = []
        self.cmd_vx = []
        self.cmd_wz = []
        self.cmd_delta = []   # Góc bẻ lái mong muốn (steering angle)

        self.odom_time = []
        self.odom_vx = []
        self.odom_wz = []
        self.odom_delta = []  # Góc bẻ lái thực tế

        # Sai số bám quỹ đạo (Cross-track Error)
        self.cte_time = []
        self.cte_values = []

        # Trạng thái
        self._start_time = None
        self._odom_threshold = 0.02
        self._last_x = None
        self._last_y = None
        
        self.L = 0.5  # Wheelbase của xe Ackermann (0.5m)

        # Thư mục lưu kết quả
        self._results_dir = os.path.expanduser('~/rover_ackermann/results')
        os.makedirs(self._results_dir, exist_ok=True)
        self._timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.get_logger().info(
            '📊 Trajectory Recorder đã khởi động.\n'
            '  - Subscribe: /odom, /plan, /cmd_vel, /amcl_pose\n'
            f'  - Kết quả sẽ lưu tại: {self._results_dir}\n'
            '  - Nhấn Ctrl+C để dừng và xuất biểu đồ.'
        )

    def _elapsed(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time is None:
            self._start_time = now
        return now - self._start_time

    def _cross_track_error(self, px, py):
        """Tính CTE: khoảng cách vuông góc ngắn nhất từ (px, py) đến đoạn thẳng trên global plan"""
        if len(self.plan_x) < 2:
            return 0.0

        plan_pts = np.column_stack((self.plan_x, self.plan_y))
        p = np.array([px, py])

        # Point to segment distance vectorized
        segments_start = plan_pts[:-1]
        segments_end = plan_pts[1:]

        line_vec = segments_end - segments_start
        pt_vec = p - segments_start

        line_len_sq = np.sum(line_vec**2, axis=1)
        line_len_sq[line_len_sq == 0] = 1e-6  # Avoid division by zero

        t = np.sum(pt_vec * line_vec, axis=1) / line_len_sq
        t = np.clip(t, 0.0, 1.0)

        projection = segments_start + t[:, np.newaxis] * line_vec
        dists = np.linalg.norm(p - projection, axis=1)

        return float(np.min(dists))

    def _odom_cb(self, msg: Odometry):
        t = self._elapsed()
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        # Calculate steering angle delta from kinematics (wz = vx * tan(delta) / L)
        # Clamp to physical limit ±0.6109 rad (35°) to avoid singularity when vx ≈ 0
        MAX_STEER = 0.6109
        delta = math.atan(wz * self.L / max(abs(vx), 0.01)) * np.sign(vx) if vx != 0 else 0.0
        delta = max(-MAX_STEER, min(MAX_STEER, delta))

        self.odom_time.append(t)
        self.odom_vx.append(vx)
        self.odom_wz.append(wz)
        self.odom_delta.append(delta)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        t = self._elapsed()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self._process_pose(x, y, yaw, t)

    def _timer_cb(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=0.05))
            t = self._elapsed()
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            self._process_pose(x, y, yaw, t)
        except Exception:
            pass

    def _process_pose(self, x, y, yaw, t):
        if self._last_x is not None:
            dist = math.sqrt((x - self._last_x) ** 2 + (y - self._last_y) ** 2)
            if dist < self._odom_threshold:
                return

        self._last_x = x
        self._last_y = y
        self.actual_x.append(x)
        self.actual_y.append(y)
        self.actual_yaw.append(yaw)
        self.actual_time.append(t)

        # CTE được tính ngay lập tức so với bản plan có sẵn tại THỜI ĐIỂM ĐÓ
        cte = self._cross_track_error(x, y)
        self.cte_time.append(t)
        self.cte_values.append(cte)

    def _plan_cb(self, msg: Path):
        self.plan_x = [p.pose.position.x for p in msg.poses]
        self.plan_y = [p.pose.position.y for p in msg.poses]
        self.get_logger().info(f'📍 Nhận Global Plan mới: {len(self.plan_x)} điểm.')

    def _cmd_vel_cb(self, msg: Twist):
        t = self._elapsed()
        vx = msg.linear.x
        wz = msg.angular.z
        
        # Clamp to physical limit ±0.6109 rad (35°) to avoid singularity when vx ≈ 0
        MAX_STEER = 0.6109
        delta = math.atan(wz * self.L / max(abs(vx), 0.01)) * np.sign(vx) if vx != 0 else 0.0
        delta = max(-MAX_STEER, min(MAX_STEER, delta))

        self.cmd_time.append(t)
        self.cmd_vx.append(vx)
        self.cmd_wz.append(wz)
        self.cmd_delta.append(delta)

    def save_csv(self):
        traj_path = os.path.join(self._results_dir, f'trajectory_{self._timestamp}.csv')
        with open(traj_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'actual_x', 'actual_y', 'actual_yaw', 'cte_m'])
            for t, x, y, yaw, cte in zip(self.actual_time, self.actual_x, self.actual_y, self.actual_yaw, self.cte_values):
                writer.writerow([f'{t:.4f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', f'{cte:.4f}'])
        print(f'[INFO] 💾 Đã lưu quỹ đạo: {traj_path}')

        if self.plan_x:
            plan_path = os.path.join(self._results_dir, f'global_plan_{self._timestamp}.csv')
            with open(plan_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['plan_x', 'plan_y'])
                for x, y in zip(self.plan_x, self.plan_y):
                    writer.writerow([f'{x:.4f}', f'{y:.4f}'])
            print(f'[INFO] 💾 Đã lưu global plan: {plan_path}')

        vel_path = os.path.join(self._results_dir, f'velocity_{self._timestamp}.csv')
        with open(vel_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'cmd_vx', 'odom_vx', 'cmd_delta', 'odom_delta'])
            if self.cmd_time and self.odom_time:
                odom_vx_interp = np.interp(self.cmd_time, self.odom_time, self.odom_vx)
                odom_delta_interp = np.interp(self.cmd_time, self.odom_time, self.odom_delta)
                for t, cv, ov, cd, od in zip(self.cmd_time, self.cmd_vx, odom_vx_interp, self.cmd_delta, odom_delta_interp):
                    writer.writerow([f'{t:.4f}', f'{cv:.4f}', f'{ov:.4f}', f'{cd:.4f}', f'{od:.4f}'])
        print(f'[INFO] 💾 Đã lưu vận tốc: {vel_path}')

    def plot_and_save(self):
        if len(self.actual_x) < 5:
            print('[WARN] Dữ liệu quá ít để vẽ biểu đồ (< 5 điểm).')
            return

        plt.style.use('default')
        ACCENT   = '#0066CC'
        PLAN_COL = '#FF6B35'
        AMCL_COL = '#2ECC71'
        CMD_COL  = '#007FFF'  # Azure Blue
        ACT_COL  = '#FF0033'  # Crimson Red
        CTE_COL  = '#E67E22'

        fig = plt.figure(figsize=(20, 14), facecolor='white')
        fig.suptitle(
            'Kết quả Bám Quỹ Đạo Xe Ackermann',
            fontsize=16, fontweight='bold', color='black', y=0.98
        )

        gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.40, wspace=0.35, left=0.06, right=0.97, top=0.93, bottom=0.07)

        # ── Biểu đồ 1: XY Quỹ đạo ─────────────
        ax1 = fig.add_subplot(gs[:, 0])
        ax1.set_facecolor('white')
        ax1.set_title('Quỹ Đạo Thực Tế', color='black', fontsize=12, fontweight='bold', pad=10)

        ax1.plot(self.actual_x, self.actual_y, color=ACCENT, linewidth=2.5, label='Quỹ đạo ', alpha=0.95, zorder=4)

        ax1.scatter([self.actual_x[0]], [self.actual_y[0]], color='#2ECC71', s=120, zorder=6, marker='o', label='Start')
        ax1.scatter([self.actual_x[-1]], [self.actual_y[-1]], color='#E74C3C', s=120, zorder=6, marker='X', label='End')

        ax1.set_xlabel('X (m)', color='black')
        ax1.set_ylabel('Y (m)', color='black')
        ax1.tick_params(colors='black')
        ax1.grid(True, color='#E0E0E0', linestyle='--', alpha=0.8)
        ax1.legend(facecolor='white', edgecolor='black', labelcolor='black', fontsize=9, loc='lower right')
        ax1.set_aspect('equal', adjustable='datalim')
        for spine in ax1.spines.values():
            spine.set_edgecolor('black')

        # ── Biểu đồ 2: Vận tốc thẳng ─────────────────────
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.set_facecolor('white')
        ax2.set_title('Vận Tốc', color='black', fontsize=11, fontweight='bold', pad=8)

        if self.cmd_time and self.odom_time:
            ax2.plot(self.cmd_time, self.cmd_vx, color=CMD_COL, linewidth=1.8, linestyle='--', label='Lệnh Điều Khiển (MPPI)', alpha=0.9)
            ax2.plot(self.odom_time, self.odom_vx, color=ACT_COL, linewidth=1.8, label='Thực tế (Odom)', alpha=0.9)

        ax2.axhline(0.8, color='#E74C3C', linestyle=':', alpha=0.5, linewidth=1, label='v_max=0.8m/s')
        ax2.axhline(-0.5, color='#E74C3C', linestyle=':', alpha=0.5, linewidth=1, label='v_min=-0.5m/s')
        ax2.set_xlabel('Thời gian (s)', color='black')
        ax2.set_ylabel('Vận tốc (m/s)', color='black')
        ax2.tick_params(colors='black')
        ax2.grid(True, color='#E0E0E0', linestyle='--', alpha=0.8)
        ax2.legend(facecolor='white', edgecolor='black', labelcolor='black', fontsize=8, loc='lower right')
        for spine in ax2.spines.values():
            spine.set_edgecolor('black')

        # ── Biểu đồ 3: Góc Bẻ Lái (Steering Angle) ───────────
        ax3 = fig.add_subplot(gs[0, 2])
        ax3.set_facecolor('white')
        ax3.set_title('Góc Bẻ Lái', color='black', fontsize=11, fontweight='bold', pad=8)

        if self.cmd_time and self.odom_time:
            ax3.plot(self.cmd_time, self.cmd_delta, color=CMD_COL, linewidth=1.8, linestyle='--', label='Lệnh Điều Khiển', alpha=0.9)
            ax3.plot(self.odom_time, self.odom_delta, color=ACT_COL, linewidth=1.8, label='Thực tế ', alpha=0.9)

        ax3.axhline(0.6109, color='#E74C3C', linestyle=':', alpha=0.5, linewidth=1, label='δ_max=35°')
        ax3.axhline(-0.6109, color='#E74C3C', linestyle=':', alpha=0.5, linewidth=1)
        ax3.set_xlabel('Thời gian (s)', color='black')
        ax3.set_ylabel('Góc (rad)', color='black')
        ax3.tick_params(colors='black')
        ax3.grid(True, color='#E0E0E0', linestyle='--', alpha=0.8)
        ax3.legend(facecolor='white', edgecolor='black', labelcolor='black', fontsize=8, loc='lower right')
        for spine in ax3.spines.values():
            spine.set_edgecolor('black')

        # ── Biểu đồ 4: Sai số bám đường (CTE) ───────────────────────────
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.set_facecolor('white')
        ax4.set_title('Sai Số Bám Đường (Cross-Track Error)', color='black', fontsize=11, fontweight='bold', pad=8)

        if self.cte_time and self.cte_values:
            cte_arr = np.array(self.cte_values)
            ax4.plot(self.cte_time, self.cte_values, color=CTE_COL, linewidth=1.8, alpha=0.9, label='CTE')
            ax4.fill_between(self.cte_time, 0, self.cte_values, alpha=0.2, color=CTE_COL)
            ax4.axhline(np.mean(cte_arr), color='black', linestyle='--', alpha=0.6, linewidth=1.2, label=f'Trung bình: {np.mean(cte_arr):.3f}m')
            ax4.axhline(0.30, color='#F39C12', linestyle=':', alpha=0.7, linewidth=1, label='Ngưỡng goal (0.30m)')

        ax4.set_xlabel('Thời gian (s)', color='black')
        ax4.set_ylabel('CTE (m)', color='black')
        ax4.set_ylim(bottom=0)
        ax4.tick_params(colors='black')
        ax4.grid(True, color='#E0E0E0', linestyle='--', alpha=0.8)
        ax4.legend(facecolor='white', edgecolor='black', labelcolor='black', fontsize=8, loc='upper right')
        for spine in ax4.spines.values():
            spine.set_edgecolor('black')

        # ── Biểu đồ 5: Thống kê tổng hợp ──────────────
        ax5 = fig.add_subplot(gs[1, 2])
        ax5.set_facecolor('white')
        ax5.set_title('Thống Kê Đánh Giá', color='black', fontsize=11, fontweight='bold', pad=8)
        ax5.axis('off')

        total_dist = 0.0
        if len(self.actual_x) > 1:
            for i in range(1, len(self.actual_x)):
                total_dist += math.sqrt((self.actual_x[i] - self.actual_x[i-1])**2 + (self.actual_y[i] - self.actual_y[i-1])**2)

        duration = self.actual_time[-1] if self.actual_time else 0.0
        avg_speed = total_dist / duration if duration > 0 else 0.0
        cte_arr = np.array(self.cte_values) if self.cte_values else np.array([0.0])

        goal_error = 0.0
        success = False
        if len(self.plan_x) > 0 and len(self.actual_x) > 0:
            goal_error = math.sqrt((self.actual_x[-1] - self.plan_x[-1])**2 + (self.actual_y[-1] - self.plan_y[-1])**2)
            success = goal_error < 0.30

        stats = [
            (' Thời gian chạy', f'{duration:.1f} s'),
            (' Tổng quãng đường', f'{total_dist:.2f} m'),
            (' Tốc độ trung bình', f'{avg_speed:.3f} m/s'),
            ('───────────────', '─────────'),
            (' Path Completion / Success', 'Đạt (YES)' if success else 'Không (NO)'),
            (' Goal Error', f'{goal_error:.3f} m'),
            ('───────────────', '─────────'),
            ('CTE Trung Bình', f'{np.mean(cte_arr):.4f} m'),
            ('CTE RMSE', f'{np.sqrt(np.mean(cte_arr**2)):.4f} m'),
            ('CTE Lớn Nhất', f'{np.max(cte_arr):.4f} m'),
        ]

        y_pos = 0.95
        for label, value in stats:
            if label.startswith('──'):
                ax5.axhline(y_pos + 0.01, color='black', linewidth=0.8, xmin=0.0, xmax=1.0)
                y_pos -= 0.07
                continue
            ax5.text(0.02, y_pos, label, transform=ax5.transAxes, fontsize=10, color='black', verticalalignment='top')
            ax5.text(0.98, y_pos, value, transform=ax5.transAxes, fontsize=10, color='black', fontweight='bold', verticalalignment='top', horizontalalignment='right')
            y_pos -= 0.08

        for spine in ax5.spines.values():
            spine.set_edgecolor('black')

        out_path = os.path.join(self._results_dir, f'trajectory_{self._timestamp}.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)

        print(f'[INFO] 📊 Biểu đồ đã lưu tại: {out_path}')

    def on_shutdown(self):
        print(f'\n[INFO] 🔴 Đang dừng... Thu được {len(self.actual_x)} điểm quỹ đạo.')
        self.save_csv()
        self.plot_and_save()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

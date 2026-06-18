#!/usr/bin/env python3
"""
Nav2 to PX4 Ackermann Rover Bridge

Converts Nav2 cmd_vel (ENU / FLU body frame) to PX4 TrajectorySetpoint (NED).
PX4 AckermannOffboardMode reads velocity mode as:
  speed      = norm(velocity_ned)
  yaw_target = atan2(v_east, v_north)
So we integrate yaw from angular.z and encode desired heading into the NED velocity direction.
"""

import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Nav2Px4Bridge(Node):
    def __init__(self):
        super().__init__('nav2_px4_bridge')

        # --- QoS Profile for PX4 Topics (BEST_EFFORT) ---
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publishers ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # --- Subscribers ---
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_px4)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos_px4)

        # --- Timer 20Hz ---
        self.timer = self.create_timer(0.05, self.timer_callback)

        # --- State ---
        self.target_speed = 0.0          # [m/s] body forward speed from Nav2
        self.target_yaw_rate = 0.0       # [rad/s] CCW positive (FLU) from Nav2
        self.desired_yaw_ned = 0.0       # [rad] integrated desired yaw in NED frame
        self.current_yaw_ned = 0.0       # [rad] measured yaw from PX4 odometry
        self.yaw_initialized = False     # wait for first odom before integrating
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.last_cmd_time = time.time()
        self.last_timer_time = time.time()
        self.counter = 0

        self.get_logger().info('Nav2→PX4 Ackermann Bridge started.')

    # ------------------------------------------------------------------
    def status_callback(self, msg):
        """Update current vehicle status."""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        """Update current NED yaw from PX4 VehicleOdometry (q = [w,x,y,z])."""
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.current_yaw_ned = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )
        # Initialize desired yaw on first odom
        if not self.yaw_initialized:
            self.desired_yaw_ned = self.current_yaw_ned
            self.yaw_initialized = True

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        """
        Receive cmd_vel from Nav2 (ENU / FLU):
          linear.x  = forward speed [m/s] (negative = reverse)
          angular.z = yaw rate CCW+ [rad/s]
        """
        self.target_speed = msg.linear.x
        raw_yaw_rate = -msg.angular.z  # Chuyển FLU sang NED

        # --- TÍNH TOÁN FEASIBLE CURVATURE ---
        L = 0.5            # Chiều dài cơ sở (wheelbase)
        max_steer = 0.6109 # Giới hạn góc bẻ lái (radian) ~ 35 độ
        k_max = math.tan(max_steer) / L

        # Xử lý an toàn khi xe đứng yên (tránh chia cho 0)
        if abs(self.target_speed) < 0.01:
            self.target_yaw_rate = 0.0
        else:
            # Tính độ cong mong muốn
            # Dùng abs(target_speed) để curvature đúng dấu khi lùi
            k_desired = raw_yaw_rate / abs(self.target_speed)
            
            # Ép độ cong vào giới hạn vật lý của xe
            k_feasible = max(min(k_desired, k_max), -k_max)
            
            # Tính lại yaw rate khả thi để điều khiển
            self.target_yaw_rate = k_feasible * abs(self.target_speed)

        self.last_cmd_time = time.time()

    # ------------------------------------------------------------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    # ------------------------------------------------------------------
    def publish_trajectory_setpoint(self, dt: float):
        # --- Safety timeout ---
        if time.time() - self.last_cmd_time > 0.5:
            self.target_speed = 0.0
            self.target_yaw_rate = 0.0

        # --- FIX INTEGRAL WINDUP: Tính toán dựa trên Yaw THỰC TẾ ---
        # Thời gian dự báo (Lookahead time) - Có thể tinh chỉnh từ 0.3s đến 1.0s
        lookahead_time = 0.5 
        
        if self.yaw_initialized:
            # Gán góc mục tiêu = Góc THỰC TẾ hiện tại + (Vận tốc góc mong muốn * Thời gian dự báo)
            self.desired_yaw_ned = self.current_yaw_ned + (self.target_yaw_rate * lookahead_time)
            # Chuẩn hóa về [-pi, pi]
            self.desired_yaw_ned = math.atan2(
                math.sin(self.desired_yaw_ned),
                math.cos(self.desired_yaw_ned)
            )

        # Build NED velocity vector
        # Nhờ PX4 đã được sửa để đọc msg.yaw trực tiếp, ta không cần ép speed tối thiểu (0.01) nữa.
        # Khi Nav2 gửi 0.0, speed sẽ bằng 0.0 và xe dừng hẳn.
        speed = self.target_speed
        
        v_north = speed * math.cos(self.desired_yaw_ned)
        v_east  = speed * math.sin(self.desired_yaw_ned)

        msg = TrajectorySetpoint()
        msg.velocity[0]    = v_north
        msg.velocity[1]    = v_east
        msg.velocity[2]    = float('nan')
        # Dù PX4 Offboard Velocity mode phớt lờ, cứ set cho chắc
        msg.yaw            = self.desired_yaw_ned
        msg.yawspeed       = self.target_yaw_rate
        msg.position[0]    = float('nan')
        msg.position[1]    = float('nan')
        msg.position[2]    = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    # ------------------------------------------------------------------
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    def timer_callback(self):
        now = time.time()
        dt = now - self.last_timer_time
        self.last_timer_time = now

        self.publish_offboard_mode()
        self.publish_trajectory_setpoint(dt)

        # Continuously try to Arm and switch to Offboard mode until successful
        # We do this every 1 second (20 ticks of the 20Hz timer)
        if self.counter % 20 == 0:
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Px4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.mode = 'MANUAL'
        self.waypoints = []
        self.active_nav_goal_handle = None
        
        self.get_logger().info('Khởi tạo Mission Manager Node...')
        
        # Action Clients tới Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.mode_sub = self.create_subscription(String, '/set_mode', self.mode_callback, 10)
        
        # Publishers
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        
        self.get_logger().info('Đang chờ Nav2 Action Servers...')
        self.nav_to_pose_client.wait_for_server()
        self.nav_through_poses_client.wait_for_server()
        self.get_logger().info('Mission Manager đã sẵn sàng! Mode hiện tại: MANUAL')
        
    def set_explore_state(self, state: bool):
        msg = Bool()
        msg.data = state
        self.explore_pub.publish(msg)
        if state:
            self.get_logger().info('[M-EXPLORE] Đã gởi lệnh RESUME/START khám phá.')
        else:
            self.get_logger().info('[M-EXPLORE] Đã gởi lệnh PAUSE dừng khám phá.')

    def cancel_nav2_goal(self):
        if self.active_nav_goal_handle is not None:
            self.get_logger().info('Đang hủy Nav2 Goal hiện tại...')
            self.active_nav_goal_handle.cancel_goal_async()
            self.active_nav_goal_handle = None

    def mode_callback(self, msg):
        new_mode = msg.data.upper()
        if new_mode not in ['MANUAL', 'WAYPOINT_RECORD', 'WAYPOINT_EXECUTE', 'EXPLORE', 'HYBRID']:
            self.get_logger().warn(f'Mode không hợp lệ: {new_mode}')
            return
            
        self.get_logger().info(f'--- CHUYỂN CHẾ ĐỘ: {self.mode} -> {new_mode} ---')
        self.mode = new_mode
        
        if self.mode == 'EXPLORE':
            self.cancel_nav2_goal()
            self.set_explore_state(True)
        elif self.mode == 'HYBRID':
            self.cancel_nav2_goal()
            self.set_explore_state(True)
        elif self.mode == 'MANUAL':
            self.set_explore_state(False)
        elif self.mode == 'WAYPOINT_RECORD':
            self.set_explore_state(False)
            self.waypoints.clear()
            self.get_logger().info('Đã xóa danh sách Waypoint. Hãy click các điểm trên RViz2.')
        elif self.mode == 'WAYPOINT_EXECUTE':
            self.set_explore_state(False)
            self.execute_waypoints()

    def goal_callback(self, msg):
        self.get_logger().info(f'Nhận tọa độ: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        
        if self.mode == 'WAYPOINT_RECORD':
            self.waypoints.append(msg)
            self.get_logger().info(f'Đã ghi nhận Waypoint thứ {len(self.waypoints)}. Đổi mode sang WAYPOINT_EXECUTE để chạy.')
            return
            
        elif self.mode == 'MANUAL':
            self.set_explore_state(False)
            self.send_single_goal(msg)
            
        elif self.mode == 'HYBRID':
            self.get_logger().info('HYBRID MODE: Tạm dừng Explore để tới điểm được chỉ định...')
            self.set_explore_state(False)
            self.send_single_goal(msg, resume_explore_on_done=True)
            
        elif self.mode == 'EXPLORE':
            self.get_logger().warn('Đang ở EXPLORE mode. Hãy chuyển sang MANUAL hoặc HYBRID để ra lệnh tay.')

    def send_single_goal(self, pose_msg, resume_explore_on_done=False):
        self.cancel_nav2_goal()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        self.get_logger().info('Đang gửi lệnh đến Nav2 (NavigateToPose)...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, resume_explore_on_done)
        )

    def execute_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('Danh sách Waypoint trống! Chuyển mode sang WAYPOINT_RECORD và click điểm trước.')
            return
            
        self.cancel_nav2_goal()
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        
        self.get_logger().info(f'Đang gửi {len(self.waypoints)} Waypoints đến Nav2 (NavigateThroughPoses)...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, resume_explore_on_done=False)
        )

    def goal_response_callback(self, future, resume_explore_on_done):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 đã TỪ CHỐI mục tiêu!')
            if resume_explore_on_done:
                self.set_explore_state(True)
            return

        self.get_logger().info('Nav2 đã CHẤP NHẬN mục tiêu. Đang di chuyển...')
        self.active_nav_goal_handle = goal_handle
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut: self.get_result_callback(fut, resume_explore_on_done)
        )

    def get_result_callback(self, future, resume_explore_on_done):
        status = future.result().status
        self.active_nav_goal_handle = None
        
        if status == 4: # SUCCEEDED
            self.get_logger().info('--- ĐÃ TỚI ĐÍCH THÀNH CÔNG! ---')
        else:
            self.get_logger().info(f'Kết thúc Nav2 Goal với status code: {status}')
            
        if resume_explore_on_done and self.mode == 'HYBRID':
            self.get_logger().info('HYBRID MODE: Đã xong nhiệm vụ. Đang tiếp tục Explore...')
            self.set_explore_state(True)


def main(args=None):
    rclpy.init(args=args)
    mission_manager = MissionManager()
    try:
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        pass
    finally:
        mission_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

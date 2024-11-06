import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
import time

class SendGoal(Node):
    def __init__(self, x, y):
        super().__init__('send_goal_node')
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        initial_x = -0.777743935585022
        initial_y = 1.3562508821487427
        initial_yaw = 1.0  

        # Set initial pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = initial_x
        initial_pose.pose.pose.position.y = initial_y
        initial_pose.pose.pose.orientation.w = 1.0  # 초기 방향 설정

        self.initial_pose_publisher_.publish(initial_pose)
        self.get_logger().info('Initial Pose Published!')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # 기본 방향 설정

        self.goal_publisher_.publish(goal)
        self.get_logger().info('Goal Published!')

class GoalAPIView(APIView):
    def post(self, request):
        try:
            x = request.data.get('x', 0.0)
            y = request.data.get('y', 0.0)

            rclpy.init(args=None)
            node = SendGoal(x, y)
            time.sleep(2)
            rclpy.shutdown()

            return Response({'status': 'Goal sent successfully'}, status=status.HTTP_200_OK)
        except Exception as e:
            return Response({'error': str(e)}, status=status.HTTP_400_BAD_REQUEST)

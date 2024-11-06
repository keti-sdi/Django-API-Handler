import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
import logging
import time

logger = logging.getLogger(__name__)

class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        
        logger.info(f"Publishing goal: x={x}, y={y}")
        self.publisher_.publish(goal)
        self.get_logger().info('Goal Published!')

# Initialize rclpy once when the Django server starts
rclpy.init(args=None)
node = SendGoal()

class GoalAPIView(APIView):
    def post(self, request):
        logger.info("Received goal request")
        try:
            x = request.data.get('x', 0.0)
            y = request.data.get('y', 0.0)
            logger.info(f"Received coordinates: x={x}, y={y}")

            # Publish the goal using the pre-initialized node
            node.publish_goal(x, y)
            
            return Response({'status': 'Goal sent successfully'}, status=status.HTTP_200_OK)
        except Exception as e:
            logger.error(f"Error occurred: {e}")
            return Response({'error': str(e)}, status=status.HTTP_400_BAD_REQUEST)

# Do not shut down rclpy until the Django server shuts down


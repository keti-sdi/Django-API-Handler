import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class SendGoal(Node):
    def __init__(self, x, y):
        super().__init__('send_goal_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Create goal message
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Set goal position (hardcoded values)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        # Publish goal
        self.publisher_.publish(goal)
        self.get_logger().info('Goal Published!')

def main():
    try:
        x = 0.029612857848405838
        y = -0.3142266571521759
        print(f"Sending goal: x={x}, y={y}")

        
        rclpy.init(args=None)
        node = SendGoal(x, y)
        time.sleep(2)
        rclpy.shutdown()

        print("Goal sent successfully!")
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == '__main__':
    main()


# ros2_core/ros2_executor.py
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from ros2_core.ros2_nodes import SendGoal, BatteryStateMonitor, TurtlebotPoseMonitor
import logging
import atexit

logger = logging.getLogger(__name__)

rclpy.init()
executor = MultiThreadedExecutor()

goal_node = SendGoal()
battery_node = BatteryStateMonitor()
pose_node = TurtlebotPoseMonitor()

executor.add_node(goal_node)
executor.add_node(battery_node)
executor.add_node(pose_node)

_executor_started = False
def start_executor():
    global _executor_started
    if not _executor_started:
        _executor_started = True  
        logger.info("Starting ROS 2 executor with multiple nodes")
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        
def stop_executor():
    executor.shutdown()
    rclpy.shutdown()

def publish_goal(x, y):
    goal_node.publish_goal(x, y)

def get_battery_state():
    return battery_node.current_battery_state

def get_pose():
    return pose_node.get_pose()
atexit.register(stop_executor)

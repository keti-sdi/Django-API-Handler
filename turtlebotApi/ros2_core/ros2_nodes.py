# ros2_core/ros2_nodes.py
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
import logging
import pika
import json
import time
import psutil
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

class BatteryStateMonitor(Node):
    def __init__(self):
        super().__init__('battery_state_monitor')
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.rabbitmq_connection = pika.BlockingConnection(pika.ConnectionParameters('10.0.5.53', 30072))
        self.rabbitmq_channel = self.rabbitmq_connection.channel()
        self.rabbitmq_channel.queue_declare(queue='metrics_data')

    def battery_callback(self, msg):
        battery_data = {
            'voltage': msg.voltage,
            'percentage': msg.percentage
        }
        cpu_usage = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        memory_usage = mem.percent
        disk = psutil.disk_usage('/')
        net_io = psutil.net_io_counters()

        system_data = {
            'cpu_usage': cpu_usage,
            'memory_usage': memory_usage,
            'disk_usage': {
                'total': disk.total,
                'used': disk.used,
                'free': disk.free,
                'percent': disk.percent
            },
            'network_io': {
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv,
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv
            }
        }

        metrics_data = {
            'battery': battery_data,
            'system': system_data
        }

        logger.info(f"터틀봇 메트릭 데이터: {metrics_data}")

        metrics_json = json.dumps(metrics_data)
        self.rabbitmq_channel.basic_publish(exchange='', routing_key='metrics_data', body=metrics_json)

        time.sleep(3)
class TurtlebotPoseMonitor(Node):
    def __init__(self):
        super().__init__('turtlebot_pose_monitor')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.current_pose = {
            'x': None,
            'y': None,
            'z': None
        }
        self.rabbitmq_connection = None
        self.rabbitmq_channel = None
        self.is_publishing = False

    def pose_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        logger.info(f"터틀봇 현재 위치: {self.current_pose}")

        if self.is_publishing and self.rabbitmq_channel:
            pose_data = json.dumps(self.current_pose)
            self.rabbitmq_channel.basic_publish(exchange='', routing_key='turtlebot_pose', body=pose_data)

    def start_publishing(self):
        if not self.is_publishing:
            self.rabbitmq_connection = pika.BlockingConnection(pika.ConnectionParameters('10.0.5.53', 30072))
            self.rabbitmq_channel = self.rabbitmq_connection.channel()
            self.rabbitmq_channel.queue_declare(queue='turtlebot_pose')
            self.is_publishing = True

            self.timer = self.create_timer(3.0, self.publish_pose_to_rabbitmq) 

    def publish_pose_to_rabbitmq(self):
        if self.rabbitmq_channel and self.is_publishing:
            pose_data = json.dumps(self.current_pose)
            self.rabbitmq_channel.basic_publish(exchange='', routing_key='turtlebot_pose', body=pose_data)
            logger.info(f"Published pose to RabbitMQ: {self.current_pose}")

    def stop_publishing(self):
        if self.is_publishing:
            self.is_publishing = False
            self.timer.cancel()  
            if self.rabbitmq_connection:
                self.rabbitmq_connection.close()

    def shutdown(self):
        self.stop_publishing()
        self.destroy_node()

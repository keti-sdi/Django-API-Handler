import rclpy
import time
import threading
import logging
import pika
import json
from django.shortcuts import render
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from django.http import JsonResponse
from django.views import View

logger = logging.getLogger('navigation')

class BatteryStateMonitor(Node):
    def __init__(self):
        super().__init__('battery_state_monitor')
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.subscription 
        self.current_battery_state = {
            'voltage': None,
            'percentage': None
        }

        #self.rabbitmq_connection = pika.BlockingConnection(pika.ConnectionParameters('rabbitmq-service')) # 파드용
        self.rabbitmq_connection = pika.BlockingConnection(pika.ConnectionParameters('10.0.5.53',30072)) # 로컬용
        self.rabbitmq_channel = self.rabbitmq_connection.channel()
        self.rabbitmq_channel.queue_declare(queue='battery_info')

    def battery_callback(self, msg):
        self.current_battery_state = {
            'voltage': msg.voltage,
            'percentage': msg.percentage
        }
        logger.info(f"Received battery info: {self.current_battery_state}")

        battery_data = json.dumps(self.current_battery_state)
        self.rabbitmq_channel.basic_publish(exchange='', routing_key='battery_info', body=battery_data)
        time.sleep(3)
        
    def shutdown(self):
        self.rabbitmq_connection.close()
        self.destroy_node()

#----배터리 정보를 구독하는 작업을 백그라운드에서 실행----#
#rclpy.init()
battery_state_monitor = BatteryStateMonitor()
subscriber_thread = threading.Thread(target=lambda: rclpy.spin(battery_state_monitor))
subscriber_thread.daemon = True  
subscriber_thread.start()

class BatteryStateView(View):
    def get(self, request, *args, **kwargs):
        battery_state = battery_state_monitor.current_battery_state
        if battery_state['voltage'] is None or battery_state['percentage'] is None:
            return JsonResponse({'error': 'Battery state not available'}, status=503)
        return JsonResponse(battery_state)

# def shutdown_ros2():
#     battery_state_monitor.shutdown()
#     rclpy.shutdown()

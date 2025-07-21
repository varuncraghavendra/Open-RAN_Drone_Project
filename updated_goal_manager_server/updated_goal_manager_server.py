#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal
import paho.mqtt.client as mqtt

class GoalManagerServer(Node):
    def __init__(self):
        super().__init__('goal_manager_server')
        self.srv = self.create_service(GetNextGoal, 'get_next_goal', self.handle_get_next_goal)
        self.get_logger().info("Goal Manager Service Ready at 'get_next_goal'")

        # Set up MQTT client to receive lat, lng updates
        self.mqtt_client = mqtt.Client()

        # Define MQTT callback methods
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # Connect to MQTT broker and subscribe to the topic
        self.mqtt_client.connect("broker.hivemq.com", 1883, 60)
        self.mqtt_client.subscribe("colosseum/update")

        # Start the MQTT loop to receive messages
        self.mqtt_client.loop_start()

        # Store current lat, lng
        self.current_lat = None
        self.current_lng = None

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker!")
        else:
            self.get_logger().error(f"Failed to connect to MQTT Broker, return code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Parse the incoming message
            message_data = json.loads(msg.payload.decode('utf-8'))
            lat = message_data['newPosition']['lat']
            lng = message_data['newPosition']['lng']

            # Update current lat, lng
            self.current_lat = lat
            self.current_lng = lng

            self.get_logger().info(f"Received new goal update: lat={lat}, lng={lng}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode message: {e}")

    def handle_get_next_goal(self, request, response):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # If lat, lng is available, use that as the goal
        if self.current_lat is not None and self.current_lng is not None:
            pose.pose.position.x = self.current_lat
            pose.pose.position.y = self.current_lng
            pose.pose.position.z = -2.0  # fly at 2 meters altitude
        else:
            # If no MQTT update received yet, generate a random goal
            pose.pose.position.x = random.uniform(-10.0, 10.0)
            pose.pose.position.y = random.uniform(-10.0, 10.0)
            pose.pose.position.z = -2.0  # e.g. fly at 2 meters altitude

        # Simple flat orientation (no rotation)
        pose.pose.orientation.w = 1.0

        response.goal_pose = pose
        self.get_logger().info(f"Generated goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#WORKING FOR MQTT

# goal_manager_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal
import paho.mqtt.client as mqtt
import json

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager_node')

        # ROS 2 Publisher to BT Navigator
        self.goal_pub = self.create_publisher(PoseStamped, '/next_goal', 10)

        # ROS 2 Client to GoalManagerService
        self.cli = self.create_client(GetNextGoal, '/get_next_goal')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_next_goal service not available, retrying...')

        # Timer for polling
        self.timer = self.create_timer(5.0, self.request_goal)

        # MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_broker_ip = '172.17.0.1'  # Replace with actual Colosseum IP if needed
        self.mqtt_port = 1883
        self.mqtt_topic = 'drone/next_goal'

        try:
            self.mqtt_client.connect(self.mqtt_broker_ip, self.mqtt_port, 60)
            self.get_logger().info(f"Connected to MQTT Broker at {self.mqtt_broker_ip}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

        self.get_logger().info("Goal Manager node started and ready.")

    def request_goal(self):
        request = GetNextGoal.Request()
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        try:
            response = future.result()
            if response and isinstance(response.goal_pose, PoseStamped):
                pose = response.goal_pose
                self.goal_pub.publish(pose)

                # Also send to MQTT broker
                goal_dict = {
                    "timestamp": str(pose.header.stamp.sec),
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                    "frame_id": pose.header.frame_id
                }
                self.mqtt_client.publish(self.mqtt_topic, json.dumps(goal_dict))
                self.get_logger().info("Published goal to /next_goal and MQTT")
            else:
                self.get_logger().warn("Received invalid goal response")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

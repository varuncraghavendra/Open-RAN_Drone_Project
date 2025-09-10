import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal
import json

BROKER = "broker.hivemq.com"
PORT = 1883
SUBSCRIBE_TOPIC = "colosseum/update"

class GoalManagerServer(Node):
    def __init__(self):
        super().__init__('goal_manager_server')

        self.latest_gps = None

        # ROS Service
        self.srv = self.create_service(GetNextGoal, 'get_next_goal', self.handle_get_next_goal)
        self.get_logger().info("Goal Manager Server started. Waiting for GPS data...")

        # MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(BROKER, PORT, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[MQTT] Connected with result code {rc}")
        client.subscribe(SUBSCRIBE_TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            lat = data.get("lat")
            lon = data.get("lon")
            self.latest_gps = (lat, lon)
            self.get_logger().info(f"Received GPS: lat={lat}, lon={lon}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse MQTT message: {e}")

    def handle_get_next_goal(self, request, response):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        if self.latest_gps:
            lat, lon = self.latest_gps
            # Here you can transform GPS to local coordinates
            pose.pose.position.x = lat
            pose.pose.position.y = lon
            pose.pose.position.z = 2.0  # e.g., fixed altitude
            self.get_logger().info(f"Using GPS as goal: lat={lat}, lon={lon}")
        else:
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 2.0
            self.get_logger().warn("No GPS data received yet, returning dummy position")

        pose.pose.orientation.w = 1.0
        response.goal_pose = pose
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

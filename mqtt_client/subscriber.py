import json
import paho.mqtt.client as mqtt

# MQTT configuration
BROKER = "broker.hivemq.com"  # Public MQTT broker for testing
PORT = 1883
UPDATE_TOPIC = "colosseum/update"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        # Subscribe to the topic after connecting
        client.subscribe(UPDATE_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        # Parse the incoming message
        message_data = json.loads(msg.payload.decode('utf-8'))
        
        # Process the message
        print("Received message:")
        print(json.dumps(message_data, indent=4))  # Pretty-print the received JSON

        # You can also access the individual fields
        print(f"Entity ID: {message_data['id']}")
        print(f"Latitude: {message_data['newPosition']['lat']}")
        print(f"Longitude: {message_data['newPosition']['lng']}")

    except json.JSONDecodeError as e:
        print(f"Failed to decode message: {e}")

# MQTT client setup
client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(BROKER, PORT, 60)

# Start the loop to listen for messages
client.loop_forever()

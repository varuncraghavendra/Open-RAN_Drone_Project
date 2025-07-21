import json
import paho.mqtt.client as mqtt
import time

# MQTT configuration
BROKER = "broker.hivemq.com"  # Public MQTT broker for testing
PORT = 1883
UPDATE_TOPIC = "colosseum/update"

def send_position_update(lat, lng, entity_id=30):
    """
    Send a position update via MQTT
    
    Args:
        lat (float): Latitude coordinate
        lng (float): Longitude coordinate  
        entity_id (int): ID of the entity to update (default: 30)
    """
    # Create MQTT client
    client = mqtt.Client(protocol=mqtt.MQTTv311)

    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {reason_code}")

    client.on_connect = on_connect

    try:
        print(f"Attempting to connect to {BROKER}:{PORT}")
        # Connect to broker
        client.connect(BROKER, PORT, 60)
        client.loop_start()

        # Prepare message in the exact format you requested
        message_data = {
            "id": entity_id,
            "dynscen_ids": [entity_id],
            "newPosition": {
                "lat": lat,
                "lng": lng
            },
            "cmd": "update",
            "type": "MQTTUpdate"
        }

        while True:
            # Send message
            result = client.publish(UPDATE_TOPIC, json.dumps(message_data))
            status = result.rc  # Check the publish result status
            if status == mqtt.MQTT_ERR_SUCCESS:
                print(f"Sent message: {json.dumps(message_data, indent=4)}")
            else:
                print(f"Failed to publish to topic {UPDATE_TOPIC}")
            
            # Sleep for 2 seconds (adjust as needed)
            time.sleep(2)
        
    except Exception as e:
        print(f"Error sending MQTT message: {e}")
    
    finally:
        client.loop_stop()
        client.disconnect()

# Example usage
if __name__ == "__main__":
    latitude = 42.33894284868896
    longitude = -71.08613491058351

    send_position_update(latitude, longitude)


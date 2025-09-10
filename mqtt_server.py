#!/usr/bin/env python3
# mqtt_server.py
# publishes raw lat/lon waypoints (no conversions)

import json
import time
import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "colosseum/update"

# Define the four GPS corners of the box (WGS84 lat, lon).
# Replace these with your exact corners if you want a different box.
POSITIONS = [
    (42.33894284868896, -71.08613491058351),
    (42.33894284868896, -71.08603491058351),
    (42.33904284868896, -71.08603491058351),
    (42.33904284868896, -71.08613491058351),
]

def main(publish_delay_s: float = 6.0):
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    idx = 0
    try:
        print("Publishing GPS box corners (lat, lon) to topic:", TOPIC)
        while True:
            lat, lon = POSITIONS[idx]
            payload = {"lat": lat, "lon": lon}   # <-- ONLY lat & lon
            client.publish(TOPIC, json.dumps(payload))
            print(f"[MQTT] published corner {idx}: lat={lat:.8f}, lon={lon:.8f}")
            idx = (idx + 1) % len(POSITIONS)
            time.sleep(publish_delay_s)
    except KeyboardInterrupt:
        print("Interrupted, stopping MQTT publisher...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()

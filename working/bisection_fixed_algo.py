#!/usr/bin/env python3
import time
import math
import json
import threading
import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt
from shapely.geometry import Polygon

# ---------------- MQTT Settings ---------------- #
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "colosseum/update"

DRONE_ID = 1

# ---------------- Fixed GPS Coordinates ---------------- #
DRONE_START_LAT = 42.33894284868896
DRONE_START_LON = -71.08613491058351

IAB_LAT = 42.3385
IAB_LON = -71.0850

# ---------------- Polygon Corners ---------------- #
FIXED_CORNERS = [
    (42.341385, -71.083002),
    (42.340315, -71.081983),
    (42.337904, -71.085191),
    (42.338665, -71.086092)
]

# ---------------- Utility Functions ---------------- #
def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS points"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def simulate_rsrp():
    """Generate dummy RSRP"""
    rsrp = -80 + (5 * math.sin(time.time()))
    return rsrp

# ---------------- MQTT Client ---------------- #
class DroneMQTTClient:
    def __init__(self):
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()

    def send_position_update(self, lat, lon, iteration, rsrp=None, distance=None):
        if rsrp is None:
            rsrp = simulate_rsrp()
        if distance is None:
            distance = haversine(lat, lon, IAB_LAT, IAB_LON)
        payload = {
            "drone_id": DRONE_ID,
            "lat": lat,
            "lon": lon,
            "rsrp": rsrp,
            "distance_to_iab": distance,
            "iteration": iteration
        }
        self.client.publish(TOPIC, json.dumps(payload))

# ---------------- Polygon Bisection ---------------- #
class PolygonDroneOptimizer:
    def __init__(self, corners, mqtt_client, dashboard=None):
        self.corners = corners
        self.mqtt_client = mqtt_client
        self.iteration = 0
        self.dashboard = dashboard

    def polygon_area(self, corners):
        poly = Polygon(corners)
        return poly.area * (111111**2)

    def run(self, min_area=100):
        current_corners = self.corners.copy()
        while True:
            self.iteration += 1
            area = self.polygon_area(current_corners)
            if area < min_area:
                print(f"[INFO] Polygon small enough (area={area:.1f} m²), stopping.")
                break

            print(f"\nIteration {self.iteration}, Polygon Area: {area:.1f} m²")
            for idx, (lat, lon) in enumerate(current_corners):
                rsrp = simulate_rsrp()
                distance = haversine(lat, lon, IAB_LAT, IAB_LON)

                print(f"Corner {idx}: lat={lat:.6f}, lon={lon:.6f} | RSRP={rsrp:.1f} dBm | "
                      f"Distance to IAB={distance:.2f} m")

                # Update dashboard
                if self.dashboard:
                    self.dashboard.update_status(lat, lon, rsrp, distance, self.iteration)

                # Publish MQTT
                self.mqtt_client.send_position_update(lat, lon, self.iteration, rsrp, distance)
                time.sleep(1)

            # Bisect polygon (keep first 2 corners, midpoint for rest)
            mid_lat = (current_corners[2][0] + current_corners[3][0]) / 2
            mid_lon = (current_corners[2][1] + current_corners[3][1]) / 2
            current_corners = [current_corners[0], current_corners[1], (mid_lat, mid_lon), (mid_lat, mid_lon)]

# ---------------- Dashboard ---------------- #
class DroneDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Drone Bisection Dashboard")
        self.root.geometry("450x300")

        self.labels = {}
        fields = ["Iteration", "Drone Lat", "Drone Lon", "IAB Lat", "IAB Lon", "RSRP", "Distance to IAB"]
        for i, field in enumerate(fields):
            tk.Label(self.root, text=field + ":").grid(row=i, column=0, sticky="w", padx=10, pady=5)
            self.labels[field] = tk.Label(self.root, text="---")
            self.labels[field].grid(row=i, column=1, sticky="w", padx=10)

        # Set fixed IAB coordinates
        self.labels["IAB Lat"].config(text=f"{IAB_LAT:.6f}")
        self.labels["IAB Lon"].config(text=f"{IAB_LON:.6f}")

    def update_status(self, lat, lon, rsrp, distance, iteration):
        self.labels["Iteration"].config(text=str(iteration))
        self.labels["Drone Lat"].config(text=f"{lat:.6f}")
        self.labels["Drone Lon"].config(text=f"{lon:.6f}")
        self.labels["RSRP"].config(text=f"{rsrp:.1f} dBm")
        self.labels["Distance to IAB"].config(text=f"{distance:.2f} m")
        self.root.update_idletasks()

    def start(self):
        self.root.mainloop()

# ---------------- Main ---------------- #
if __name__ == "__main__":
    mqtt_client = DroneMQTTClient()
    dashboard = DroneDashboard()

    optimizer = PolygonDroneOptimizer(FIXED_CORNERS, mqtt_client, dashboard=dashboard)

    # Run optimizer in a separate thread so the Tkinter dashboard remains responsive
    t = threading.Thread(target=optimizer.run)
    t.start()

    # Start dashboard main loop
    dashboard.start()

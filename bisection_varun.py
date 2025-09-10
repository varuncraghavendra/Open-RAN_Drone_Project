#!/usr/bin/env python3
import time
import math
import json
import tkinter as tk
from tkinter import ttk
import threading

import paho.mqtt.client as mqtt
from shapely.geometry import Polygon

# ---------------- MQTT Settings ---------------- #
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "colosseum/update"

# IAB fixed GPS coordinates
IAB_LAT = 42.3385
IAB_LON = -71.0850
DRONE_ID = 1

# ---------------- Custom Polygon ---------------- #
CUSTOM_BOUNDS = {
    'top_left': [42.34138538637752, -71.08300209045412],
    'top_right': [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left': [42.33866528158443, -71.08609199523927]
}

def create_custom_polygon():
    coords = [
        (CUSTOM_BOUNDS['top_left'][1], CUSTOM_BOUNDS['top_left'][0]),
        (CUSTOM_BOUNDS['top_right'][1], CUSTOM_BOUNDS['top_right'][0]),
        (CUSTOM_BOUNDS['bottom_right'][1], CUSTOM_BOUNDS['bottom_right'][0]),
        (CUSTOM_BOUNDS['bottom_left'][1], CUSTOM_BOUNDS['bottom_left'][0])
    ]
    return coords  # list of (lng, lat)

def get_polygon_area_meters(corners):
    poly = Polygon(corners)
    return poly.area * (111111**2)

def get_midpoint(p1, p2):
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

def cut_polygon_in_half(corners, best_edge_idx):
    c_idx = best_edge_idx
    d_idx = (best_edge_idx+1)%4
    a_idx = (best_edge_idx+2)%4
    b_idx = (best_edge_idx+3)%4
    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]
    mid_ad = get_midpoint(point_a, point_d)
    mid_bc = get_midpoint(point_b, point_c)
    new_corners = [point_c, point_d, mid_bc, mid_ad]
    return new_corners

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2-lat1)
    dlambda = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*c

# ---------------- MQTT Drone Client ---------------- #
class DroneMQTTClient:
    def __init__(self):
        self._client = mqtt.Client()
        self._client.on_connect = self.on_connect
        self._client.on_message = self.on_message
        self._client.connect(BROKER, PORT, 60)
        self._client.loop_start()
        self.current_lat = None
        self.current_lon = None

    def on_connect(self, client, userdata, flags, rc):
        print("[MQTT] Connected with result code", rc)
        client.subscribe(TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            self.current_lat = float(data.get("lat"))
            self.current_lon = float(data.get("lon"))
        except Exception as e:
            print("Error parsing MQTT message:", e)

# ---------------- Dashboard ---------------- #
class DroneDashboard:
    def __init__(self, optimizer):
        self.optimizer = optimizer
        self.root = tk.Tk()
        self.root.title("Drone Polygon Bisection Dashboard")
        self.root.geometry("850x400")
        self.tree = ttk.Treeview(
            self.root, columns=("drone_id","lat","lng","rsrp","ues","distance"), show="headings"
        )
        for col, width in zip(["drone_id","lat","lng","rsrp","ues","distance"],
                              [80,120,120,100,100,120]):
            self.tree.heading(col, text=col.capitalize())
            self.tree.column(col, width=width)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.update_dashboard()

    def update_dashboard(self):
        for i in self.tree.get_children():
            self.tree.delete(i)
        for lat, lng, rsrp, ues, reason, iteration in self.optimizer.trajectory[-15:]:
            distance = haversine(lat, lng, IAB_LAT, IAB_LON)
            self.tree.insert(
                "", "end",
                values=(DRONE_ID, f"{lat:.6f}", f"{lng:.6f}", f"{rsrp:.1f}", ues, f"{distance:.2f}")
            )
        self.root.after(1000, self.update_dashboard)

    def run(self):
        self.root.mainloop()

# ---------------- Drone Optimizer ---------------- #
class PolygonDroneOptimizer:
    def __init__(self, initial_corners, mqtt_client=None):
        self.initial_corners = initial_corners
        self.mqtt_client = mqtt_client
        self.trajectory = []
        self.current_position = None
        self.iteration = 0

    def move_drone_step_by_step(self, target_lat, target_lng, reason="movement"):
        if self.current_position is None:
            print(f"Teleporting drone to start: ({target_lat:.6f},{target_lng:.6f})")
            if self.mqtt_client:
                self.mqtt_client._client.publish(
                    TOPIC, json.dumps({"lat": target_lat,"lon":target_lng,"drone_id":DRONE_ID})
                )
            self.current_position = (target_lat,target_lng)
            time.sleep(0.2)
            return
        cur_lat, cur_lng = self.current_position
        steps = 5
        for i in range(1, steps+1):
            lat = cur_lat + (target_lat - cur_lat)*i/steps
            lng = cur_lng + (target_lng - cur_lng)*i/steps
            if self.mqtt_client:
                self.mqtt_client._client.publish(
                    TOPIC, json.dumps({"lat": lat,"lon":lng,"drone_id":DRONE_ID})
                )
            time.sleep(0.2)
        self.current_position = (target_lat,target_lng)

    def measure_rsrp_real(self, lat, lng, reason="measurement"):
        self.move_drone_step_by_step(lat, lng, reason)
        rsrp = -80 + 5*math.sin(time.time())
        connected_ues = int(abs(rsrp+80))%5+1
        self.trajectory.append((lat,lng,rsrp,connected_ues,reason,self.iteration))
        print(f"Measurement ({lat:.6f},{lng:.6f}) -> RSRP={rsrp:.1f}, UEs={connected_ues}")
        return rsrp

    def polygon_bisection_search_real(self, min_area_threshold=100):
        print("Starting polygon bisection...")
        current_corners = self.initial_corners.copy()
        measured_positions = {}
        while True:
            self.iteration += 1
            area = get_polygon_area_meters(current_corners)
            print(f"Iteration {self.iteration}, polygon area: {area:.1f} m²")
            if area < min_area_threshold:
                print("Polygon small enough, stopping.")
                break
            corner_measurements = []
            for i,(lng,lat) in enumerate(current_corners):
                pos_key = (round(lat,8), round(lng,8))
                if pos_key in measured_positions:
                    metric = measured_positions[pos_key][0]
                else:
                    metric = self.measure_rsrp_real(lat, lng, f"corner_{i}")
                    measured_positions[pos_key] = (metric,1)
                corner_measurements.append(metric)
            edge_scores = [(corner_measurements[i]+corner_measurements[(i+1)%4],i) for i in range(4)]
            best_edge_idx = max(edge_scores,key=lambda x:x[0])[1]
            current_corners = cut_polygon_in_half(current_corners,best_edge_idx)
        print("Bisection finished.")
        return current_corners[0][1], current_corners[0][0]

# ---------------- Main ---------------- #
if __name__ == "__main__":
    print("Starting bisection algorithm with live dashboard and MQTT subscription...")
    mqtt_client = DroneMQTTClient()
    initial_corners = create_custom_polygon()
    optimizer = PolygonDroneOptimizer(initial_corners, mqtt_client)
    dashboard = DroneDashboard(optimizer)
    # Run bisection in background
    threading.Thread(target=lambda: optimizer.polygon_bisection_search_real(), daemon=True).start()
    dashboard.run()

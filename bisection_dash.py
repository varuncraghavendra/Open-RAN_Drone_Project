import requests
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import numpy as np
import networkx as nx
import pickle
import os
from hashlib import md5
import time
import tikzplotlib
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import paho.mqtt.client as mqtt
import random

from influx_hook import get_average_rsrp_3s
from mqtt_client import send_position_update

CENTER_LAT = 42.33894284868896
CENTER_LNG = -71.08613491058351
RADIUS_METERS = 1000
GRID_SPACING_METERS = 5

CUSTOM_BOUNDS = {
    'top_left': [42.34138538637752, -71.08300209045412],
    'top_right': [42.340314805239075, -71.08198285102846],
    'bottom_right': [42.337903948379996, -71.08519077301027],
    'bottom_left': [42.33866528158443, -71.08609199523927]
}

def get_cache_filename(cache_type="buildings"):
    key = f"{CENTER_LAT}_{CENTER_LNG}_{RADIUS_METERS}"
    if cache_type == "graph":
        key += f"_grid{GRID_SPACING_METERS}m"
    hash_key = md5(key.encode()).hexdigest()[:8]
    return f"{cache_type}_cache_{hash_key}.pkl"

def meters_to_degrees(meters, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return meters / (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return meters / 111111.0

def degrees_to_meters(degrees, is_longitude=False, center_lat=None):
    if is_longitude and center_lat is not None:
        return degrees * (111111.0 * np.cos(np.radians(center_lat)))
    else:
        return degrees * 111111.0

def create_custom_polygon():
    coords = [
        (CUSTOM_BOUNDS['top_left'][1], CUSTOM_BOUNDS['top_left'][0]),
        (CUSTOM_BOUNDS['top_right'][1], CUSTOM_BOUNDS['top_right'][0]),
        (CUSTOM_BOUNDS['bottom_right'][1], CUSTOM_BOUNDS['bottom_right'][0]),
        (CUSTOM_BOUNDS['bottom_left'][1], CUSTOM_BOUNDS['bottom_left'][0])
    ]
    return coords

def get_polygon_area_meters(corners):
    poly = Polygon(corners)
    area_sq_meters = poly.area * (111111**2)
    return area_sq_meters

def get_midpoint(point1, point2):
    return ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

def cut_polygon_in_half(corners, best_edge_idx):
    c_idx = best_edge_idx
    d_idx = (best_edge_idx + 1) % 4
    a_idx = (best_edge_idx + 2) % 4
    b_idx = (best_edge_idx + 3) % 4

    point_c = corners[c_idx]
    point_d = corners[d_idx]
    point_a = corners[a_idx]
    point_b = corners[b_idx]

    mid_ad = get_midpoint(point_a, point_d)
    mid_bc = get_midpoint(point_b, point_c)

    new_corners = [point_c, point_d, mid_bc, mid_ad]
    return new_corners

def load_graph():
    cache_file = get_cache_filename("graph")
    buildings_cache = get_cache_filename("buildings")

    print(f"Loading graph from cache: {cache_file}")
    with open(cache_file, 'rb') as f:
        cached_data = pickle.load(f)

    print(f"Loading buildings from cache: {buildings_cache}")
    with open(buildings_cache, 'rb') as f:
        buildings = pickle.load(f)

    return buildings, cached_data['grid'], cached_data['graph'], cached_data['key_to_node']

def get_real_rsrp_metric_robust():
    time.sleep(1)
    return random.uniform(-100, -60), random.randint(1, 5)

def find_nearest_graph_node(G, target_lat, target_lng):
    min_distance = float('inf')
    nearest_node = None

    for node in G.nodes():
        node_lat = G.nodes[node]['lat']
        node_lng = G.nodes[node]['lng']
        distance = np.sqrt((node_lat - target_lat)**2 + (node_lng - target_lng)**2)
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node

def get_shortest_path_between_points(G, lat1, lng1, lat2, lng2):
    node1 = find_nearest_graph_node(G, lat1, lng1)
    node2 = find_nearest_graph_node(G, lat2, lng2)

    if node1 is None or node2 is None:
        return None, None

    try:
        path_nodes = nx.shortest_path(G, node1, node2, weight='weight')
        path_coords = [(G.nodes[node]['lat'], G.nodes[node]['lng']) for node in path_nodes]
        path_length = nx.shortest_path_length(G, node1, node2, weight='weight')
        return path_coords, path_length
    except nx.NetworkXNoPath:
        print(f"No path found between ({lat1:.6f}, {lng1:.6f}) and ({lat2:.6f}, {lng2:.6f})")
        return None, None

class PolygonDroneOptimizer:
    def __init__(self, initial_corners, graph, dashboard, entity_id=30):
        self.initial_corners = initial_corners
        self.graph = graph
        self.entity_id = entity_id
        self.trajectory = []
        self.full_path = []
        self.iteration = 0
        self.current_position = None
        self.dashboard = dashboard

    def move_drone_step_by_step(self, target_lat, target_lng, reason="movement"):
        if self.current_position is None:
            print(f"Teleporting drone to start position: ({target_lat:.6f}, {target_lng:.6f})")
            send_position_update(target_lat, target_lng, self.entity_id)
            self.current_position = (target_lat, target_lng)
            self.full_path.append((target_lat, target_lng, reason))
            time.sleep(0.15)
            return

        current_lat, current_lng = self.current_position
        path_coords, path_length = get_shortest_path_between_points(
            self.graph, current_lat, current_lng, target_lat, target_lng
        )

        if path_coords and len(path_coords) > 1:
            print(f"Moving drone via {len(path_coords)} waypoints to {reason}")
            for i, (lat, lng) in enumerate(path_coords):
                if i == 0:
                    continue
                send_position_update(lat, lng, self.entity_id)
                path_reason = f"travel_to_{reason}"
                if i == len(path_coords) - 1:
                    path_reason = reason
                self.full_path.append((lat, lng, path_reason))
                time.sleep(0.5)
            self.current_position = (target_lat, target_lng)
        else:
            print(f"Warning: No path found to target, using direct movement")
            send_position_update(target_lat, target_lng, self.entity_id)
            self.full_path.append((target_lat, target_lng, reason))
            self.current_position = (target_lat, target_lng)
            time.sleep(0.05)

    def measure_rsrp_real(self, lat, lng, reason="measurement"):
        print(f"\n--- Moving to {reason} at ({lat:.6f}, {lng:.6f}) ---")
        self.move_drone_step_by_step(lat, lng, reason)
        print("Waiting 0.5s for drone to stabilize...")
        time.sleep(0.5)
        print("Taking RSRP measurement...")
        metric, connected_ues = get_real_rsrp_metric_robust()
        distance_m = random.uniform(100, 500)
        self.trajectory.append((lat, lng, metric, connected_ues, reason, self.iteration))
        print(f"Measurement complete: metric={metric:.1f}, connected_UEs={connected_ues}")
        self.dashboard.update_status(lat, lng, metric, self.entity_id, distance_m)
        return metric

    def polygon_bisection_search_real(self, min_area_threshold=100):
        print("Starting polygon bisection...")
        current_corners = self.initial_corners.copy()
        measured_positions = {}
        corner_labels = ["corner_0", "corner_1", "corner_2", "corner_3"]

        while True:
            self.iteration += 1
            print(f"\nITERATION {self.iteration}")
            area_sq_meters = get_polygon_area_meters(current_corners)
            print(f"Current polygon area: {area_sq_meters:.1f} sq meters")

            if area_sq_meters < min_area_threshold:
                print("Polygon is small enough, stopping search.")
                break

            for i, (lng, lat) in enumerate(current_corners):
                print(f"  Corner {i}: ({lat:.6f}, {lng:.6f})")

            corner_measurements = []
            if self.iteration == 1:
                for i, (lng, lat) in enumerate(current_corners):
                    pos_key = (round(lat, 8), round(lng, 8))
                    if pos_key in measured_positions:
                        cached_metric, cached_ues = measured_positions[pos_key]
                        print(f"  {corner_labels[i]}: Using cached -> Metric: {cached_metric:.1f}")
                        corner_measurements.append(cached_metric)
                    else:
                        metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                        corner_measurements.append(metric)
                        last_measurement = self.trajectory[-1]
                        measured_positions[pos_key] = (last_measurement[2], last_measurement[3])
            else:
                for i, (lng, lat) in enumerate(current_corners):
                    pos_key = (round(lat, 8), round(lng, 8))
                    if i < 2:
                        if pos_key in measured_positions:
                            cached_metric, cached_ues = measured_positions[pos_key]
                            print(f"  {corner_labels[i]}: Using cached -> Metric: {cached_metric:.1f}")
                            corner_measurements.append(cached_metric)
                        else:
                            metric = self.measure_rsrp_real(lat, lng, corner_labels[i])
                            corner_measurements.append(metric)
                            last_measurement = self.trajectory[-1]
                            measured_positions[pos_key] = (last_measurement[2], last_measurement[3])
                    else:
                        metric = self.mea

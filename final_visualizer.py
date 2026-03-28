import pyvista as pv
import numpy as np
import csv
import time
import sys
from collections import defaultdict

SPEED_SCALE = 0.012
DODGE_RADIUS = 0.5     
ARRIVAL_DIST = 0.08
MISSION_DIST = 1.8
WORLD_SIZE = 10.0
OCTREE_CAPACITY = 8      
pv.global_theme.multi_samples = 0

def load_uav_data(max_drones: int):
    rows = []
    with open("uav_data.csv", "r") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            try:
                float(row[2])
                rows.append(row)
                if len(rows) >= max_drones:
                    break
            except Exception:
                pass
    data = np.array([[float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])] for row in rows], dtype = float,)
    mn = data[:, :3].min(axis=0)
    mx = data[:, :3].max(axis=0)
    bounds = {"min": mn, "max": mx}
    return data, bounds

def normalize(coords, bounds):
    return (coords - bounds["min"]) / (bounds["max"] - bounds["min"] + 1e-9) * WORLD_SIZE

def build_positions_and_targets(raw, bounds):
    n = len(raw)
    positions = normalize(raw[:, :3], bounds).copy()
    targets = []
    for i in range(n):
        yaw = np.radians(raw[i, 4])
        pitch = np.radians(raw[i, 5])
        d = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch),]) 
        t = positions[i] + d * MISSION_DIST
        targets.append(np.clip(t, 0.1, WORLD_SIZE - 0.1))
    return positions, np.array(targets)

def brute_force_collisions(positions, active, radius):
    n = len(positions)
    repulsion = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    for i in range(n):
        if not active[i]:
            continue
        for j in range(i + 1, n):
            if not active[j]:
                continue
            diff = positions[i] - positions[j]
            d = np.linalg.norm(diff)
            if d < radius:
                force = (diff / (d + 1e-9)) * (radius - d)
                repulsion[i] += force
                repulsion[j] -= force
                is_dodging[i] = True
                is_dodging[j] = True
    return repulsion, is_dodging

class OctreeNode:
    def __init__(self, cx: float, cy: float, cz: float, hs: float):
        self.cx, self.cy, self.cz = cx, cy, cz
        self.hs = hs
        self.indices = []         
        self.children = [None] * 8        
        self.is_leaf = True

    def contains(self, px, py, pz) -> bool:
        return (self.cx - self.hs <= px <= self.cx + self.hs and self.cy - self.hs <= py <= self.cy + self.hs and self.cz - self.hs <= pz <= self.cz + self.hs)

    def intersects_sphere(self, sx, sy, sz, r) -> bool:
        dx = max(self.cx - self.hs - sx, 0.0, sx - (self.cx + self.hs))
        dy = max(self.cy - self.hs - sy, 0.0, sy - (self.cy + self.hs))
        dz = max(self.cz - self.hs - sz, 0.0, sz - (self.cz + self.hs))
        return dx * dx + dy * dy + dz * dz <= r * r
    
    def _subdivide(self, positions: np.ndarray):
        q = self.hs / 2.0
        self.children = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    self.children.append(OctreeNode(self.cx + sx * q, self.cy + sy * q, self.cz + sz * q, q))
        self.is_leaf = False
        for idx in self.indices:
            p = positions[idx]
            for child in self.children:
                if child.contains(p[0], p[1], p[2]):
                    child.indices.append(idx)
                    break
        self.indices = []

    def insert(self, drone_idx: int, positions: np.ndarray, capacity: int):
        p = positions[drone_idx]
        if not self.contains(p[0], p[1], p[2]):
            return False
        if self.is_leaf:
            self.indices.append(drone_idx)
            if len(self.indices) > capacity:
                self._subdivide(positions)
            return True
        for child in self.children:
            if child.insert(drone_idx, positions, capacity):
                return True
        return False

    def query_radius(self, sx, sy, sz, r, result: list):
        if not self.intersects_sphere(sx, sy, sz, r):
            return
        if self.is_leaf:
            result.extend(self.indices)
        else:
            for child in self.children:
                child.query_radius(sx, sy, sz, r, result)

    def collect_boxes(self, out: list):
        out.append((self.cx, self.cy, self.cz, self.hs))
        if not self.is_leaf:
            for child in self.children:
                child.collect_boxes(out)

def build_octree(positions: np.ndarray, active: np.ndarray) -> OctreeNode:
    half = WORLD_SIZE / 2.0
    root = OctreeNode(half, half, half, half)
    for i in np.where(active)[0]:
        root.insert(i, positions, OCTREE_CAPACITY)
    return root

def octree_collisions(positions, active, radius, root):
    n = len(positions)
    repulsion = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    checked = set()
    for i in range(n):
        if active[i]:
            p = positions[i]
            neighbours = []
            root.query_radius(p[0], p[1], p[2], radius, neighbours)
            for j in neighbours:
                if j == i:
                    continue
                pair = (min(i, j), max(i, j))
                if pair in checked:
                    continue
                checked.add(pair)
                diff = positions[i] - positions[j]
                d = np.linalg.norm(diff)
                if d < radius:
                    force = (diff / (d + 1e-9)) * (radius - d)
                    repulsion[i] += force
                    repulsion[j] -= force
                    is_dodging[i] = True
                    is_dodging[j] = True
    return repulsion, is_dodging

def step_physics(positions, targets, status, repulsion, is_dodging):
    n = len(positions)
    for i in range(n):
        if status[i] == 1:
            continue
        diff = targets[i] - positions[i]
        dist = np.linalg.norm(diff)
        move = (diff / (dist + 1e-9)) * SPEED_SCALE
        positions[i] += move + repulsion[i]
        new_dist = np.linalg.norm(targets[i] - positions[i])
        if new_dist < ARRIVAL_DIST:
            status[i] = 1
        elif is_dodging[i]:
            status[i] = 2
        else:
            status[i] = 0
    # 1 = arrive, 2 = dodge, 0 = traveling

def run_simulation(mode: str, num_drones: int):
    raw, bounds = load_uav_data(num_drones)
    actual = len(raw)
    positions, targets = build_positions_and_targets(raw, bounds)
    status    = np.zeros(actual)
    dodge_count = 0
    collision_times = []
    build_times = []
    frame_times = []
    plotter = pv.Plotter(title = f"UAV Swarm — {'Brute Force' if mode == 'brute' else 'Octree'} ({actual} drones)")
    plotter.set_background("#0d0d0d")
    box = pv.Cube(center = (WORLD_SIZE / 2,)*3, x_length = WORLD_SIZE, y_length = WORLD_SIZE, z_length = WORLD_SIZE)
    plotter.add_mesh(box, style = "wireframe", color = "white", opacity = 0.15, line_width = 1)
    plane = pv.Plane(center = (WORLD_SIZE / 2, WORLD_SIZE / 2, 0), direction = (0, 0, 1), i_size = WORLD_SIZE, j_size = WORLD_SIZE, i_resolution = 10, j_resolution = 10)
    plotter.add_mesh(plane, style = "wireframe", color = "gray", opacity = 0.1)
    drone_cloud = pv.PolyData(positions[:, [1, 0, 2]])
    drone_cloud["status"] = status
    drone_actor = plotter.add_mesh(drone_cloud, render_points_as_spheres = True, point_size = 12, scalars = "status", cmap = ["cyan", "lime", "red"], clim = [0, 2], show_scalar_bar = False,)
    target_cloud = pv.PolyData(targets[:, [1, 0, 2]])
    plotter.add_mesh(target_cloud, color = "orange", point_size = 5, render_points_as_spheres = True, opacity = 0.4)
    octree_actors = []
    hud = plotter.add_text("", position = "upper_left", color = "white", font_size = 11, shadow = True)
    plotter.show(interactive_update=True, auto_close=False)
    while True:
        if not plotter.ren_win:         
            break
        t_frame_start = time.perf_counter()
        active = status != 1
        build_ms = 0.0
        octree_root = None
        if mode == "brute":
            t_col_start = time.perf_counter()
            repulsion, is_dodging = brute_force_collisions(positions, active, DODGE_RADIUS)
            t_col_end = time.perf_counter()
            collision_ms = (t_col_end - t_col_start) * 1000.0
        else:
            t_build_start = time.perf_counter()
            octree_root = build_octree(positions, active)
            t_build_end = time.perf_counter()
            t_col_start = time.perf_counter()
            repulsion, is_dodging = octree_collisions(positions, active, DODGE_RADIUS, octree_root)
            t_col_end = time.perf_counter()
            build_ms = (t_build_end - t_build_start) * 1000.0
            collision_ms = (t_col_end - t_col_start) * 1000.0
        dodge_count += int(np.sum(is_dodging))
        step_physics(positions, targets, status, repulsion, is_dodging) 
        if mode == "octree" and octree_root is not None:
            for a in octree_actors:
                plotter.remove_actor(a)
            octree_actors.clear()
            boxes: list = []
            octree_root.collect_boxes(boxes)
            for (cx, cy, cz, hs) in boxes:
                if hs < WORLD_SIZE / 4:
                    cube = pv.Cube(center = (cy * 1, cx * 1, cz * 1), x_length = hs * 2, y_length = hs * 2, z_length = hs * 2,)
                    a = plotter.add_mesh(cube, style = "wireframe", color = "deepskyblue", opacity = 0.06, line_width = 1)
                    octree_actors.append(a)
        drone_cloud.points = positions[:, [1, 0, 2]]
        drone_cloud["status"] = status
        collision_times.append(collision_ms)
        build_times.append(build_ms)
        avg_collision = sum(collision_times) / len(collision_times)
        avg_build = sum(build_times) / len(build_times)
        reached = int(np.sum(status == 1))
        flying = actual - reached
        dodging_now = int(np.sum(status == 2))
        if mode == "brute":
            timing_lines = f"Collision Query: {avg_collision:.2f}ms"
        else:
            timing_lines = f"Tree build: {avg_build:.2f} ms\nCollision query: {avg_collision:.2f} ms\nBuild + query: {avg_build + avg_collision:.2f} ms"
        hud.SetText(
            0,
            f"Active Drones :  {flying}\n"
            f"Reached Target:  {reached}\n"
            f"Dodging Now   :  {dodging_now}\n"
            f"Total Dodges  :  {dodge_count // 2}\n"
            f"──────────────────────────\n"
            f"{timing_lines}"
        )
        plotter.update()
        elapsed = time.perf_counter() - t_frame_start
        time.sleep(max(0.0, 0.016 - elapsed))

def get_user_inputs():
    print("UAV Swarm — Collision Avoidance Sim")
    print("1. Brute Force visualisation")
    print("2. Octree visualisation")
    mode_input = input("Select mode [1/2]: ").strip()
    mode_map = {"1": "brute", "2": "octree"}
    mode = mode_map.get(mode_input, "brute")
    try:
        n = int(input("Number of drones: ").strip())
    except ValueError:
        n = 60
    return mode, n

mode, n = get_user_inputs()
run_simulation(mode, n)
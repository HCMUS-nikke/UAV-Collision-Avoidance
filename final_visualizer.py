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
    data = np.array([[float(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6]), float(r[7])] for r in rows], dtype = float,)
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
        d = np.array([np.cos(pitch) * np.cos(yaw),np.cos(pitch) * np.sin(yaw),np.sin(pitch),])
        t = positions[i] + d * MISSION_DIST
        targets.append(np.clip(t, 0.1, WORLD_SIZE - 0.1))
    return positions, np.array(targets)

def brute_force_collisions(positions: np.ndarray, active: np.ndarray, radius: float):
    n = len(positions)
    repulsion = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    idx = np.where(active)[0]
    for ii, i in enumerate(idx):
        for j in idx[ii + 1:]:
            diff = positions[i] - positions[j]
            d = np.linalg.norm(diff)
            if d < radius:
                force = (diff / (d + 1e-9)) * (radius - d) * 0.4
                repulsion[i] += force
                repulsion[j] -= force
                is_dodging[i] = True
                is_dodging[j] = True
    return repulsion, is_dodging

class OctreeNode:
    __slots__ = ("cx", "cy", "cz", "hs", "indices", "children", "is_leaf")
    def __init__(self, cx: float, cy: float, cz: float, hs: float):
        self.cx, self.cy, self.cz = cx, cy, cz
        self.hs = hs
        self.indices = []         
        self.children = None        
        self.is_leaf = True

    def contains(self, px, py, pz) -> bool:
        return ( self.cx - self.hs <= px <= self.cx + self.hs and self.cy - self.hs <= py <= self.cy + self.hs and self.cz - self.hs <= pz <= self.cz + self.hs)

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

def octree_collisions(positions: np.ndarray, active: np.ndarray, radius: float, root: OctreeNode):
    n = len(positions)
    repulsion  = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    checked = set()
    for i in np.where(active)[0]:
        p = positions[i]
        neighbours: list = []
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
                force = (diff / (d + 1e-9)) * (radius - d) * 0.4
                repulsion[i] += force
                repulsion[j] -= force
                is_dodging[i] = True
                is_dodging[j] = True
    return repulsion, is_dodging

def step_physics(positions, targets, status, repulsion, is_dodging):
    active = status != 1
    diff = targets - positions
    dist = np.linalg.norm(diff, axis=1, keepdims=True)
    move = np.where(active[:, None], diff / (dist + 1e-9) * SPEED_SCALE, 0.0)
    positions[active] += move[active] + repulsion[active]
    arrived_dist = np.linalg.norm(targets - positions, axis = 1)
    status[(status != 1) & (arrived_dist < ARRIVAL_DIST)] = 1  
    status[(status != 1) & is_dodging] = 2  
    status[(status != 1) & ~is_dodging] = 0  
    # 1 = arrive, 2 = dodge, 0 = traveling

def run_benchmark(num_drones: int, frames: int = 120):
    print(f"\n{'─'*55}")
    print(f"  BENCHMARK  |  {num_drones} drones  |  {frames} frames")
    print(f"{'─'*55}")
    raw, bounds = load_uav_data(num_drones)
    actual = len(raw)
    pos_bf, tgt_bf = build_positions_and_targets(raw, bounds)
    status_bf = np.zeros(actual)
    bf_times = []
    for _ in range(frames):
        active = status_bf != 1
        t0 = time.perf_counter()
        rep, dodging = brute_force_collisions(pos_bf, active, DODGE_RADIUS)
        bf_times.append(time.perf_counter() - t0)
        step_physics(pos_bf, tgt_bf, status_bf, rep, dodging)
    pos_oc, tgt_oc = build_positions_and_targets(raw, bounds)
    status_oc = np.zeros(actual)
    oc_times = []
    build_times = []
    for _ in range(frames):
        active = status_oc != 1
        t0 = time.perf_counter()
        root = build_octree(pos_oc, active)
        build_times.append(time.perf_counter() - t0)
        t1 = time.perf_counter()
        rep, dodging = octree_collisions(pos_oc, active, DODGE_RADIUS, root)
        oc_times.append(time.perf_counter() - t1)
        step_physics(pos_oc, tgt_oc, status_oc, rep, dodging)
    bf_mean = np.mean(bf_times)  * 1000
    oc_mean = (np.mean(oc_times) + np.mean(build_times)) * 1000
    q_mean  = np.mean(oc_times)  * 1000
    bd_mean = np.mean(build_times) * 1000
    speedup = bf_mean / oc_mean if oc_mean > 0 else float("inf")
    print(f"  {'Metric':<30} {'Brute Force':>10}  {'Octree':>10}")
    print(f"  {'─'*53}")
    print(f"  {'Mean time / frame (ms)':<30} {bf_mean:>10.3f}  {oc_mean:>10.3f}")
    print(f"  {'  └─ collision query (ms)':<30} {'':>10}  {q_mean:>10.3f}")
    print(f"  {'  └─ tree build (ms)':<30} {'':>10}  {bd_mean:>10.3f}")
    print(f"  {'Total over all frames (ms)':<30} {sum(bf_times)*1000:>10.1f}  {(sum(oc_times)+sum(build_times))*1000:>10.1f}")
    print(f"  {'Speedup (Octree vs BF)':<30} {'':>10}  {speedup:>9.2f}x")
    print(f"{'─'*55}\n")

def run_simulation(mode: str, num_drones: int):
    raw, bounds = load_uav_data(num_drones)
    actual = len(raw)
    positions, targets = build_positions_and_targets(raw, bounds)
    status    = np.zeros(actual)
    dodge_count = 0
    plotter = pv.Plotter(title = f"UAV Swarm — {'Brute Force' if mode == 'brute' else 'Octree'} ({actual} drones)")
    plotter.set_background("#0d0d0d")
    box = pv.Cube(center = (WORLD_SIZE/2,)*3, x_length = WORLD_SIZE, y_length = WORLD_SIZE, z_length = WORLD_SIZE)
    plotter.add_mesh(box, style = "wireframe", color = "white", opacity = 0.15, line_width = 1)
    plane = pv.Plane(center = (WORLD_SIZE / 2, WORLD_SIZE / 2, 0), direction = (0, 0, 1), i_size = WORLD_SIZE, j_size = WORLD_SIZE, i_resolution = 10, j_resolution = 10)
    plotter.add_mesh(plane, style = "wireframe", color = "gray", opacity = 0.1)
    drone_cloud  = pv.PolyData(positions[:, [1, 0, 2]])
    drone_cloud["status"] = status
    drone_actor = plotter.add_mesh(drone_cloud, render_points_as_spheres = True, point_size = 12, scalars = "status", cmap = ["cyan", "lime", "red"], clim = [0, 2], show_scalar_bar = False,)
    target_cloud = pv.PolyData(targets[:, [1, 0, 2]])
    plotter.add_mesh(target_cloud, color = "orange", point_size = 5, render_points_as_spheres = True, opacity = 0.4)
    octree_actors = []
    hud = plotter.add_text("", position = "upper_left", color = "white", font_size = 11, shadow = True)
    plotter.add_text("", position = "lower_left", color = "yellow", font_size = 10, shadow=True)
    frame_times = []
    plotter.show(interactive_update=True, auto_close=False)
    while True:
        if not plotter.ren_win:         
            break
        t0 = time.perf_counter()
        active = status != 1
        if mode == "brute":
            repulsion, is_dodging = brute_force_collisions(positions, active, DODGE_RADIUS)
            octree_root = None
        else:
            octree_root = build_octree(positions, active)
            repulsion, is_dodging = octree_collisions(positions, active, DODGE_RADIUS, octree_root)
        dodge_count += int(np.sum(is_dodging))
        step_physics(positions, targets, status, repulsion, is_dodging)
        frame_ms = (time.perf_counter() - t0) * 1000
        frame_times.append(frame_ms)
        avg_ms = np.mean(frame_times[-60:])  
        if mode == "octree" and octree_root is not None:
            for a in octree_actors:
                plotter.remove_actor(a)
            octree_actors.clear()
            boxes: list = []
            octree_root.collect_boxes(boxes)
            for (cx, cy, cz, hs) in boxes:
                if hs < WORLD_SIZE / 4:
                    cube = pv.Cube(
                        center = (cy * 1, cx * 1, cz * 1),
                        x_length = hs * 2, y_length = hs * 2, z_length = hs * 2,
                    )
                    a = plotter.add_mesh(cube, style="wireframe", color ="deepskyblue", opacity =0.06, line_width=1)
                    octree_actors.append(a)
        drone_cloud.points = positions[:, [1, 0, 2]]
        drone_cloud["status"] = status
        reached = int(np.sum(status == 1))
        flying = actual - reached
        dodging_now = int(np.sum(status == 2))
        hud.SetText(
            0,
            f"Active Drones :  {flying}\n"
            f"Reached Target:  {reached}\n"
            f"Dodging Now   :  {dodging_now}\n"
            f"Total Dodges  :  {dodge_count // 2}\n"
            f"──────────────────\n"
            f"Frame time    :  {frame_ms:5.1f} ms\n"
            f"Avg (60f)     :  {avg_ms:5.1f} ms\n"
            f"Est. FPS      :  {10/avg_ms:5.1f}",
        )
        plotter.update()
        time.sleep(max(0.0, 0.016 - (time.perf_counter() - t0)))

def get_user_inputs():
    print("UAV Swarm — Collision Avoidance Sim")
    print("1. Brute Force visualisation")
    print("2. Octree visualisation")
    print("3. Benchmark")
    mode_input = input("Select mode [1/2/3]: ").strip()
    mode_map = {"1": "brute", "2": "octree", "3": "benchmark"}
    mode = mode_map.get(mode_input, "brute")
    try:
        n = int(input("Number of drones: ").strip())
    except ValueError:
        n = 60
    return mode, n

if __name__ == "__main__":
    mode, n = get_user_inputs()
    if mode == "benchmark":
        frames = 120
        run_benchmark(n, frames)
    else:
        run_simulation(mode, n)
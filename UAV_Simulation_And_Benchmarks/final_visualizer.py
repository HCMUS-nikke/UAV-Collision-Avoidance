import pyvista as pv
import numpy as np
import csv
import time
import sys
import tracemalloc
import matplotlib.pyplot as plt
from collections import defaultdict
import os
from pathlib import Path

SPEED_SCALE = 0.012
DODGE_RADIUS = 0.5     
COLLISION_RADIUS = 0.1 
ARRIVAL_DIST = 0.08
MISSION_DIST = 1.8
WORLD_SIZE = 10.0
OCTREE_CAPACITY = 8      
pv.global_theme.multi_samples = 0

def load_or_generate_uav_data(max_drones: int):
    rows = []
    try:
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
    except FileNotFoundError:
        print("Error: uav_data.csv not found.")
        sys.exit(1)
        
    data = np.array([[float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])] for row in rows], dtype = float,)
    
    actual_drones = len(data)
    if actual_drones < max_drones and actual_drones > 0:
        print(f"Generating {max_drones - actual_drones} synthetic drones to meet requested scale...")
        synthetic = []
        for _ in range(max_drones - actual_drones):
            base_idx = np.random.randint(0, actual_drones)
            base = data[base_idx].copy()
            base[0] += np.random.uniform(-0.02, 0.02)
            base[1] += np.random.uniform(-0.02, 0.02)
            base[2] += np.random.uniform(-10, 10)
            synthetic.append(base)
        data = np.vstack((data, np.array(synthetic)))

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

def brute_force_collisions(positions, active, radius, collision_rad):
    n = len(positions)
    repulsion = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    collisions_count = 0
    idx = np.where(active)[0]
    for ii, i in enumerate(idx):
        for j in idx[ii + 1:]:
            diff = positions[i] - positions[j]
            d = np.linalg.norm(diff)
            if d < radius:
                if d < collision_rad:
                    collisions_count += 1
                force = (diff / (d + 1e-9)) * (radius - d)
                repulsion[i] += force
                repulsion[j] -= force
                is_dodging[i] = True
                is_dodging[j] = True
    return repulsion, is_dodging, collisions_count, 0

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

    def query_radius(self, sx, sy, sz, r, result: list, metrics: dict):
        if metrics is not None:
            metrics["queries"] += 1
        if not self.intersects_sphere(sx, sy, sz, r):
            return
        if self.is_leaf:
            r_sq = r * r
            for idx in self.indices:
                px, py, pz = self.points[idx]     
                ex, ey, ez = px - sx, py - sy, pz - sz
                if ex*ex + ey*ey + ez*ez <= r_sq: 
                    result.append(idx)
        else:
            for child in self.children:
                if child is not None:
                    child.query_radius(sx, sy, sz, r, result, metrics)

    def collect_boxes(self, out: list):
        out.append((self.cx, self.cy, self.cz, self.hs))
        if not self.is_leaf:
            for child in self.children:
                if child is not None:
                    child.collect_boxes(out)

def build_octree(positions: np.ndarray, active: np.ndarray, capacity: int) -> OctreeNode:
    half = WORLD_SIZE / 2.0
    root = OctreeNode(half, half, half, half)
    for i in np.where(active)[0]:
        root.insert(i, positions, capacity)
    return root

def octree_collisions(positions, active, radius, collision_rad, root, metrics):
    n = len(positions)
    repulsion = np.zeros_like(positions)
    is_dodging = np.zeros(n, dtype=bool)
    checked = set()
    collisions_count = 0
    false_positives = 0
    
    idx = np.where(active)[0]
    for i in idx:
        p = positions[i]
        neighbours = []
        root.query_radius(p[0], p[1], p[2], radius, neighbours, metrics)
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
                if d < collision_rad:
                    collisions_count += 1
                force = (diff / (d + 1e-9)) * (radius - d)
                repulsion[i] += force
                repulsion[j] -= force
                is_dodging[i] = True
                is_dodging[j] = True
            else:
                false_positives += 1
                
    return repulsion, is_dodging, collisions_count, false_positives

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

def create_benchmark_folder(folder_name='benchmark_results'):
    Path(folder_name).mkdir(parents=True, exist_ok=True)
    return folder_name

def run_simulation(mode: str, num_drones: int):
    raw, bounds = load_or_generate_uav_data(num_drones)
    actual = len(raw)
    positions, targets = build_positions_and_targets(raw, bounds)
    status    = np.zeros(actual)
    dodge_count = 0
    collision_times = []
    build_times = []
    
    plotter = pv.Plotter(title = f"UAV Swarm Visualizer - {mode} ({actual} drones)")
    plotter.set_background("#0d0d0d")
    box = pv.Cube(center = (WORLD_SIZE / 2,)*3, x_length = WORLD_SIZE, y_length = WORLD_SIZE, z_length = WORLD_SIZE)
    plotter.add_mesh(box, style = "wireframe", color = "white", opacity = 0.15, line_width = 1)
    plane = pv.Plane(center = (WORLD_SIZE / 2, WORLD_SIZE / 2, 0), direction = (0, 0, 1), i_size = WORLD_SIZE, j_size = WORLD_SIZE, i_resolution = 10, j_resolution = 10)
    plotter.add_mesh(plane, style = "wireframe", color = "gray", opacity = 0.1)
    
    drone_cloud = pv.PolyData(positions[:, [1, 0, 2]])
    drone_cloud["status"] = status
    plotter.add_mesh(drone_cloud, render_points_as_spheres = True, point_size = 12, scalars = "status", cmap = ["cyan", "lime", "red"], clim = [0, 2], show_scalar_bar = False,)
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
            repulsion, is_dodging, _, _ = brute_force_collisions(positions, active, DODGE_RADIUS, COLLISION_RADIUS)
            t_col_end = time.perf_counter()
            collision_ms = (t_col_end - t_col_start) * 1000.0
        else:
            t_build_start = time.perf_counter()
            octree_root = build_octree(positions, active, OCTREE_CAPACITY)
            t_build_end = time.perf_counter()
            t_col_start = time.perf_counter()
            repulsion, is_dodging, _, _ = octree_collisions(positions, active, DODGE_RADIUS, COLLISION_RADIUS, octree_root, None)
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

def run_headless_simulation(mode, num_drones, frames, capacity, clustered=False):
    raw, bounds = load_or_generate_uav_data(num_drones)
    positions, targets = build_positions_and_targets(raw, bounds)
    
    if clustered:
        positions = positions * 0.15 + (WORLD_SIZE * 0.42)
        
    status = np.zeros(len(raw))
    
    total_time = 0
    total_dodges = 0
    total_collisions = 0
    total_fps = 0
    metrics = {"queries": 0}
    
    frame_times = []
    build_times = []
    query_times = []

    tracemalloc.start()
    
    for _ in range(frames):
        active = status != 1
        t0 = time.perf_counter()
        
        if mode == "brute":
            rep, dodging, frame_cols, _ = brute_force_collisions(positions, active, DODGE_RADIUS, COLLISION_RADIUS)
            frame_time = time.perf_counter() - t0
            frame_times.append(frame_time * 1000)
            query_times.append(frame_time * 1000)
            build_times.append(0)
        else:
            tb = time.perf_counter()
            root = build_octree(positions, active, capacity)
            tb_end = time.perf_counter()
            
            tq = time.perf_counter()
            rep, dodging, frame_cols, fps = octree_collisions(positions, active, DODGE_RADIUS, COLLISION_RADIUS, root, metrics)
            tq_end = time.perf_counter()
            
            frame_time = tq_end - tb
            frame_times.append(frame_time * 1000)
            build_times.append((tb_end - tb) * 1000)
            query_times.append((tq_end - tq) * 1000)
            total_fps += fps
            
        step_physics(positions, targets, status, rep, dodging)
        total_time += frame_time
        total_dodges += int(np.sum(dodging))
        total_collisions += frame_cols

    current_mem, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return {
        "time_ms": total_time * 1000,
        "peak_ram_mb": peak_mem / (1024 * 1024),
        "dodges": total_dodges,  
        "collisions": total_collisions,
        "queries": metrics["queries"],
        "false_positives": total_fps,
        "frame_times": frame_times,
        "build_times": build_times,
        "query_times": query_times
    }

def plot_comparison(bf_res, oc_res, folder):
    labels = ['Brute Force', 'Octree']
    colors = ['salmon', 'skyblue']
    
    plt.figure(figsize=(8, 6))
    plt.bar(labels, [bf_res['time_ms'], oc_res['time_ms']], color=colors, alpha=0.8)
    plt.title('Total Execution Time (ms)')
    plt.ylabel('Time (ms)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/time_comparison.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.bar(labels, [bf_res['peak_ram_mb'], oc_res['peak_ram_mb']], color=colors, alpha=0.8)
    plt.title('Peak RAM Usage (MB)')
    plt.ylabel('RAM (MB)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/ram_comparison.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.bar(labels, [bf_res['dodges'], oc_res['dodges']], color=colors, alpha=0.8)
    plt.title('Dodges Triggered (Drones)')
    plt.ylabel('Total Drones Dodging')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/dodges_comparison.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.bar(labels, [bf_res['collisions'], oc_res['collisions']], color=colors, alpha=0.8)
    plt.title('Collisions Occurred (Pairs)')
    plt.ylabel('Total Pairwise Collisions')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/collisions_comparison.png")
    plt.close()

def plot_capacity_impact(capacities, times, rams, queries, collisions, folder):
    optimal_cap = capacities[times.index(min(times))]
    min_time = min(times)
    
    plt.figure(figsize=(8, 6))
    plt.plot(capacities, times, marker='o', color='red')
    plt.axvline(x=optimal_cap, color='green', linestyle='--', alpha=0.7)
    plt.text(optimal_cap, min_time, f' Optimal: {optimal_cap}', color='green', va='bottom', ha='right')
    plt.title('Impact of Max Capacity on Execution Time')
    plt.xlabel('Max Capacity per Block')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.savefig(f"{folder}/capacity_time_impact.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.plot(capacities, rams, marker='s', color='blue')
    plt.title('Impact of Max Capacity on Peak RAM Usage')
    plt.xlabel('Max Capacity per Block')
    plt.ylabel('RAM (MB)')
    plt.grid(True)
    plt.savefig(f"{folder}/capacity_ram_impact.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.plot(capacities, queries, marker='^', color='green')
    plt.title('Impact of Max Capacity on Octree Queries')
    plt.xlabel('Max Capacity per Block')
    plt.ylabel('Total Queries')
    plt.grid(True)
    plt.savefig(f"{folder}/capacity_queries_impact.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    plt.plot(capacities, collisions, marker='d', color='purple')
    plt.title('Impact of Max Capacity on Collisions')
    plt.xlabel('Max Capacity per Block')
    plt.ylabel('Collisions Count')
    plt.grid(True)
    plt.savefig(f"{folder}/capacity_collisions_impact.png")
    plt.close()

def plot_advanced_report(folder, scaling_data, cluster_data, var_data, overhead_data, false_positive_data):
    drones = [d['drones'] for d in scaling_data]
    bf_times = [d['bf_time'] for d in scaling_data]
    oc_times = [d['oc_time'] for d in scaling_data]

    plt.figure(figsize=(8, 6))
    plt.plot(drones, bf_times, marker='o', color='red', label='Brute Force O(n^2)')
    plt.plot(drones, oc_times, marker='^', color='blue', label='Octree O(n log n)')
    plt.title('Algorithm Scaling Comparison (Full Scale)')
    plt.xlabel('Number of Drones')
    plt.ylabel('Execution Time (ms)')
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{folder}/scaling_curve_full.png")
    plt.close()

    low_drones = [d['drones'] for d in scaling_data if d['drones'] <= 50]
    low_bf = [d['bf_time'] for d in scaling_data if d['drones'] <= 50]
    low_oc = [d['oc_time'] for d in scaling_data if d['drones'] <= 50]

    plt.figure(figsize=(8, 6))
    plt.plot(low_drones, low_bf, marker='o', color='red', label='Brute Force')
    plt.plot(low_drones, low_oc, marker='^', color='blue', label='Octree')
    plt.title('Low-Scale Intersection (0-50 Drones)')
    plt.xlabel('Number of Drones')
    plt.ylabel('Execution Time (ms)')
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{folder}/scaling_curve_crossover.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    labels = ['Uniform Data', 'Clustered Data']
    colors = ['lightgreen', 'orange']
    plt.bar(labels, [cluster_data['uni_time'], cluster_data['clus_time']], color=colors)
    plt.title('The Clustering Penalty (Execution Time)')
    plt.ylabel('Time (ms)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/clustering_penalty.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    drones_oh = [d['drones'] for d in overhead_data]
    builds = [np.mean(d['builds']) for d in overhead_data]
    queries = [np.mean(d['queries']) for d in overhead_data]
    plt.bar(drones_oh, builds, label='Tree Build Time', color='salmon', width=3)
    plt.bar(drones_oh, queries, bottom=builds, label='Query Time', color='skyblue', width=3)
    plt.title('Octree Overhead: Build vs Query')
    plt.xlabel('Number of Drones')
    plt.ylabel('Average Time per Frame (ms)')
    plt.legend()
    plt.savefig(f"{folder}/build_vs_query.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    box_data = [var_data['bf_frames'], var_data['oc_frames']]
    plt.boxplot(box_data, labels=['Brute Force', 'Octree'])
    plt.title('Frame Time Variance and 1% Lows Analysis')
    plt.ylabel('Frame Time (ms)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/variance_analysis.png")
    plt.close()

    plt.figure(figsize=(8, 6))
    fp_labels = ['True Dodges', 'False Positives', 'Hard Collisions']
    fp_values = [false_positive_data['dodges'], false_positive_data['fps'], false_positive_data['cols']]
    plt.bar(fp_labels, fp_values, color=['cyan', 'gray', 'red'])
    plt.title('Octree Accuracy and Collision Tracking (400 Drones)')
    plt.ylabel('Total Occurrences')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.savefig(f"{folder}/accuracy_and_collisions.png")
    plt.close()

def run_benchmark_suite():
    print("\nHeadless Benchmark Suite")
    print("1. Standard Benchmark")
    print("2. Varied Benchmark")
    bench_type = input("Select benchmark [1/2]: ").strip()
    
    print("\nSelect Algorithm:")
    print("1. Octree")
    print("2. Brute Force")
    print("3. Both")
    algo_choice = input("Select algorithm [1/2/3]: ").strip()
    
    is_varied = (bench_type == "2")
    algos = ["octree"] if algo_choice == "1" else ["brute"] if algo_choice == "2" else ["brute", "octree"]
    
    frames = 120
    folder = create_benchmark_folder()
    
    if not is_varied:
        try:
            drones = int(input("Number of drones (default 100): ").strip() or "100")
        except ValueError:
            drones = 100
            
        capacity = 8
        if "octree" in algos:
            try:
                capacity = int(input("Octree Max Capacity (default 8): ").strip() or "8")
            except ValueError:
                capacity = 8
                
        bf_res = None
        oc_res = None
        
        if "brute" in algos:
            print("\nRunning Brute Force...")
            bf_res = run_headless_simulation("brute", drones, frames, capacity)
            print(f"Brute Force Time: {bf_res['time_ms']:.2f} ms | RAM: {bf_res['peak_ram_mb']:.4f} MB")
            
        if "octree" in algos:
            print(f"\nRunning Octree (Capacity: {capacity})...")
            oc_res = run_headless_simulation("octree", drones, frames, capacity)
            print(f"Octree Time: {oc_res['time_ms']:.2f} ms | RAM: {oc_res['peak_ram_mb']:.4f} MB | Queries: {oc_res['queries']}")
            
        if bf_res and oc_res:
            plot_comparison(bf_res, oc_res, folder)
            print(f"Standard comparison charts saved to {folder}/")
            
    else:
        print("\nRunning Varied Benchmark Suite...")
            
        if algo_choice == "1" or algo_choice == "3": 
            print("\nRunning Octree Max Capacity Optimizer (200 drones)...")
            test_capacities = [2, 4, 6, 8, 10, 12, 16, 20, 24, 32, 48, 64]
            times, rams, queries, collisions = [], [], [], []
            for cap in test_capacities:
                res = run_headless_simulation("octree", 200, frames, cap)
                times.append(res['time_ms'])
                rams.append(res['peak_ram_mb'])
                queries.append(res['queries'])
                collisions.append(res['collisions'])
                
            optimal_cap = test_capacities[times.index(min(times))]
            print(f"\n[+] Optimal Capacity: {optimal_cap} drones/block (Time: {min(times):.2f} ms)")
                
            plot_capacity_impact(test_capacities, times, rams, queries, collisions, folder)
            
            print("\nRunning Clustering Penalty Test (using 400 drones)...")
            uni = run_headless_simulation("octree", 400, frames, optimal_cap, clustered=False)
            clus = run_headless_simulation("octree", 400, frames, optimal_cap, clustered=True)
            plt.figure(figsize=(8, 6))
            plt.bar(['Uniform', 'Clustered'], [uni['time_ms'], clus['time_ms']], color=['lightgreen', 'orange'])
            plt.title('The Clustering Penalty (Execution Time)')
            plt.ylabel('Time (ms)')
            plt.savefig(f"{folder}/clustering_penalty.png")
            plt.close()
            print(f"Varied Octree charts saved to {folder}/")
            
        if algo_choice == "2" or algo_choice == "3": 
            print("\nRunning Scaling Curve (Brute Force)...")
            scale_steps = [5, 10, 20, 50, 100, 200, 400]
            bf_times = []
            for d in scale_steps:
                res = run_headless_simulation("brute", d, frames, 8)
                bf_times.append(res['time_ms'])
            plt.figure(figsize=(8, 6))
            plt.plot(scale_steps, bf_times, marker='o', color='red', label='Brute Force O(n^2)')
            plt.title('Brute Force Scaling Curve')
            plt.xlabel('Number of Drones')
            plt.ylabel('Execution Time (ms)')
            plt.legend()
            plt.grid(True)
            plt.savefig(f"{folder}/bf_scaling_curve.png")
            plt.close()
            print(f"Brute Force scaling curve saved to {folder}/bf_scaling_curve.png")
            
        if algo_choice == "3": 
            print("\nRunning Varied Benchmark...")
            scale_steps = [5, 10, 20, 50, 100, 200, 400]
            scaling_data = []
            overhead_data = []
            for d in scale_steps:
                print(f"Testing {d} drones...")
                bf = run_headless_simulation("brute", d, frames, 8)
                oc = run_headless_simulation("octree", d, frames, 8)
                scaling_data.append({'drones': d, 'bf_time': bf['time_ms'], 'oc_time': oc['time_ms']})
                overhead_data.append({'drones': d, 'builds': oc['build_times'], 'queries': oc['query_times']})
            
            print("\nRunning Variance and False Positive Analysis (400 drones)...")
            bf_var = run_headless_simulation("brute", 400, frames, 8)
            oc_var = run_headless_simulation("octree", 400, frames, 8)
            var_data = {'bf_frames': bf_var['frame_times'], 'oc_frames': oc_var['frame_times']}
            
            fp_data = {'dodges': oc_var['dodges'], 'fps': oc_var['false_positives'], 'cols': oc_var['collisions']}
            
            plot_advanced_report(folder, scaling_data, {'uni_time': uni['time_ms'], 'clus_time': clus['time_ms']}, var_data, overhead_data, fp_data)
            print(f"\nAdvanced varied comparison charts saved to {folder}/")

def get_user_inputs():
    print("UAV Swarm Collision Avoidance Sim")
    print("1. Brute Force visualisation")
    print("2. Octree visualisation")
    print("3. Headless Benchmark Suite")
    
    mode_input = input("Select mode [1/2/3]: ").strip()
    
    if mode_input == "3":
        return "benchmark", 0
        
    mode_map = {"1": "brute", "2": "octree"}
    mode = mode_map.get(mode_input, "brute")
    try:
        n = int(input("Number of drones: ").strip())
    except ValueError:
        n = 60
    return mode, n

if __name__ == "__main__":
    mode, n = get_user_inputs()
    if mode == "benchmark":
        run_benchmark_suite()
    else:
        run_simulation(mode, n)

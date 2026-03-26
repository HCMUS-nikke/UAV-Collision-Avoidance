import pyvista as pv
import numpy as np
import csv
import argparse
import sys
import time
pv.global_theme.multi_samples = 0

def load_uav_data():
    uav_rows = []
    try:
        with open('uav_data.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader) 
            for row in reader:
                try:
                    float(row[2])
                    uav_rows.append(row)
                except: pass
    except FileNotFoundError:
        print("Error: uav_data.csv not found.")
        sys.exit(1)

    data = np.array([[float(r[2]), float(r[3]), float(r[4]), float(r[5])] for r in uav_rows])
    lats, lons, alts = data[:, 0], data[:, 1], data[:, 2]
    
    bounds = {
        'min': np.min(data[:, :3], axis=0),
        'max': np.max(data[:, :3], axis=0)
    }
    return data, bounds

def normalize_vec(coords, bounds):
    return (coords - bounds['min']) / (bounds['max'] - bounds['min'])

def run_octree_mode(data, bounds):
    num_drones = len(data)
    real_coords = data[:, :3].copy()
    speeds = (data[:, 3] / 111320.0) * 0.5 
    
    status = np.zeros(num_drones)

    target_lat = bounds['min'][0] + (bounds['max'][0] - bounds['min'][0]) * 0.8
    target_lon = bounds['min'][1] + (bounds['max'][1] - bounds['min'][1]) * 0.8
    target_alt = bounds['min'][2] + (bounds['max'][2] - bounds['min'][2]) * 0.8
    target_vec = np.array([target_lat, target_lon, target_alt])

    plotter = pv.Plotter()
    plotter.set_background('white')

    try:
        with open('octree_boxes.csv', 'r') as f:
            reader = csv.reader(f); next(reader)
            for row in reader:
                nx, ny, nz, hs = map(float, row)
                cube = pv.Cube(center=(nx, ny, nz), x_length=hs*2, y_length=hs*2, z_length=hs*2)
                plotter.add_mesh(cube, style='wireframe', color='black', opacity=0.15)
    except: pass

    norm_coords = normalize_vec(real_coords, bounds)
    point_cloud = pv.PolyData(norm_coords[:, [1, 0, 2]]) # Lon, Lat, Alt mapping
    point_cloud['status'] = status
    plotter.add_mesh(pv.Sphere(center=(0.8, 0.8, 0.8), radius=0.02), color='red')
    plotter.add_mesh(point_cloud, render_points_as_spheres=True, point_size=12, 
                     scalars='status', cmap=['blue', 'green', 'red'], show_scalar_bar=False)

    plotter.show(interactive_update=True)

    last_hud_update = 0

    while True:
        loop_start = time.time()
        
        diffs = target_vec - real_coords
        dists = np.linalg.norm(diffs, axis=1)
        
        active_mask = (status == 0) & (dists > 0.005)
        if np.any(active_mask):
            real_coords[active_mask] += (diffs[active_mask] / dists[active_mask, None]) * speeds[active_mask, None]
            point_cloud.points = normalize_vec(real_coords, bounds)[:, [1, 0, 2]]
        
        status[(status == 0) & (dists <= 0.005)] = 1
        if time.time() - last_hud_update > 0.05: # Throttle HUD/Collision logic for performance
            active_idx = np.where(status == 0)[0]
            for i in active_idx:
                others = active_idx[active_idx > i]
                if len(others) == 0: continue
                
                d_diff = np.abs(real_coords[i] - real_coords[others])
                collisions = (d_diff[:, 0] < 0.02) & (d_diff[:, 1] < 0.02) & (d_diff[:, 2] < 100.0)
                
                if np.any(collisions):
                    status[i] = 2
                    status[others[collisions]] = 2
            reached = np.sum(status == 1)
            total_coll = np.sum(status == 2)
            hud = f"OCTREE MODE\nReached: {int(reached)}\nCollisions: {int(total_coll)}"
            plotter.add_text(hud, position='upper_left', color='black', font_size=12, name='HUD')
            last_hud_update = time.time()

        point_cloud['status'] = status
        plotter.camera.azimuth += 0.3
        plotter.update()
        time.sleep(max(0, 0.016 - (time.time() - loop_start)))

def run_brute_mode(bounds):
    frames = {}
    try:
        with open('brute_force_frames.csv', 'r') as f:
            reader = csv.reader(f); next(reader) 
            for row in reader:
                f_idx = int(row[0])
                if f_idx not in frames: frames[f_idx] = []
                frames[f_idx].append(row)
    except:
        print("Run C++ first."); sys.exit(1)

    max_f = max(frames.keys())
    plotter = pv.Plotter()
    plotter.set_background('white')
    plotter.add_mesh(pv.Cube(center=(0.5, 0.5, 0.5)), style='wireframe', color='black', line_width=2)
    f0_data = np.array([[float(r[2]), float(r[3]), float(r[4]), float(r[5])] for r in frames[0]])
    norm_f0 = normalize_vec(f0_data[:, :3], bounds)
    point_cloud = pv.PolyData(norm_f0[:, [1, 0, 2]])
    point_cloud['status'] = f0_data[:, 3]
    
    plotter.add_mesh(point_cloud, render_points_as_spheres=True, point_size=12, 
                     scalars='status', cmap=['blue', 'red'], show_scalar_bar=False)
    collision_cube = pv.Cube(x_length=0.03, y_length=0.03, z_length=0.03)
    glyphs = point_cloud.glyph(geom=collision_cube, orient=False, scale=False)
    plotter.add_mesh(glyphs, style='wireframe', color='black', opacity=0.3, name='boxes')

    plotter.show(interactive_update=True)

    curr_f = 0
    while True:
        f_data = np.array([[float(r[2]), float(r[3]), float(r[4]), float(r[5])] for r in frames[curr_f]])
        
        norm_update = normalize_vec(f_data[:, :3], bounds)
        point_cloud.points = norm_update[:, [1, 0, 2]]
        point_cloud['status'] = f_data[:, 3]
        new_g = point_cloud.glyph(geom=collision_cube, orient=False, scale=False)
        plotter.add_mesh(new_g, style='wireframe', color='black', opacity=0.3, name='boxes')

        coll_count = np.sum(f_data[:, 3] == 1)
        hud = f"BRUTE FORCE\nFrame: {curr_f}\nCollisions: {int(coll_count)}"
        plotter.add_text(hud, position='upper_left', color='black', font_size=12, name='HUD')

        curr_f = (curr_f + 1) % (max_f + 1)
        plotter.camera.azimuth += 0.3
        plotter.update()
        time.sleep(0.05)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, choices=['octree', 'brute'], default='octree')
    args = parser.parse_args()
    
    uav_data, uav_bounds = load_uav_data()
    if args.mode == 'octree':
        run_octree_mode(uav_data, uav_bounds)
    else:
        run_brute_mode(uav_bounds)

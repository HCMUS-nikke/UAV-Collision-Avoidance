import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import pandas as pd
import numpy as np
import random

# --- Configuration ---
SPEED = 0.08
PADDING = 0.6
CSV_FILE = 'uav_data.csv'
MAX_DRONES = 100 
VIEW_BOX = 15.0 

class UAV:
    def __init__(self, start_pos, target_pos, uav_id):
        self.id = uav_id
        self.pos = np.array(start_pos, dtype=float)
        self.target = np.array(target_pos, dtype=float)
        self.finished = False
        
        # Unique speed for each drone based on the data
        self.speed = SPEED * (random.uniform(0.8, 1.2))

    def update(self, all_uavs):
        if self.finished:
            return

        # Trajectory towards target
        direction = self.target - self.pos
        dist_to_target = np.linalg.norm(direction)

        if dist_to_target < 0.3:
            self.pos = np.copy(self.target)
            self.finished = True
            return

        move_vector = direction / (dist_to_target + 0.001)

        # Dodge Mechanism
        repulsion = np.zeros(3)
        for other in all_uavs:
            if other is self or other.finished:
                continue
            
            diff = self.pos - other.pos
            dist = np.linalg.norm(diff)
            
            if dist < PADDING:
                repulsion += (diff / (dist + 0.001)) * (PADDING - dist)

        final_dir = move_vector + (repulsion * 2.0)
        mag = np.linalg.norm(final_dir)
        
        if mag > 0:
            self.pos += (final_dir / mag) * self.speed

    def draw(self):
        glPointSize(6)
        glBegin(GL_POINTS)
        if self.finished:
            glColor3f(0.0, 1.0, 0.0) 
        else:
            glColor3f(0.0, 0.8, 1.0) 
        glVertex3fv(self.pos)
        glEnd()
        glPointSize(3)
        glBegin(GL_POINTS)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3fv(self.target)
        glEnd()


def normalize_data(array, scale):
    min_val, max_val = np.min(array), np.max(array)
    if max_val == min_val: 
        return np.zeros_like(array)
    return ((array - min_val) / (max_val - min_val) * (2 * scale)) - scale # Normalization formula


def load_uavs_from_csv(filename):
    df = pd.read_csv(filename).head(MAX_DRONES)
    
    # Normalize Lat, Long, and Alt to fit inside VIEW_BOX
    x_coords = normalize_data(df['Latitude'].values, VIEW_BOX)
    y_coords = normalize_data(df['Longitude'].values, VIEW_BOX)
    z_coords = normalize_data(df['Altitude'].values, VIEW_BOX)
    
    uavs = []
    for i in range(len(df)):
        start = [x_coords[i], y_coords[i], z_coords[i]]
        
        target = [
            random.uniform(-VIEW_BOX, VIEW_BOX), 
            random.uniform(-VIEW_BOX, VIEW_BOX), 
            random.uniform(-VIEW_BOX, VIEW_BOX)
        ]
        uavs.append(UAV(start, target, df['UAV_ID'].iloc[i]))
        
    return uavs


def main():
    pygame.init()
    display = (1280, 720)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    
    gluPerspective(45, (display[0]/display[1]), 0.1, 200.0)
    glTranslatef(0, 0, -60) 

    uav_swarm = load_uavs_from_csv(CSV_FILE)
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        
        glRotatef(0.2, 0, 1, 0)

        all_finished = True
        for uav in uav_swarm:
            uav.update(uav_swarm)
            uav.draw()
            if not uav.finished:
                all_finished = False

        if all_finished:
            pass 

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
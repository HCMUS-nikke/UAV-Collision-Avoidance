# UAV Collision Avoidance

## Introduction
- Our project is about predicting and preventing drone collisions using the Octree algorithm.
- We feel that this topic in general is pretty cool, relevant, and applicable in many problems such as during game developing, and including day to day ones (especially in the middle east 💀). 
- Our members also do not have any prior experience with these kinds of algorithms so we chose it for our research.

- We've chosen Octree and raw Brute force for our project. Octree is especially important here as without this algorithm, we would have to query the distance between one drone with n-1 other drones leading to having to do n^2 calculatiosn per query. Due to the time it takes for one query, when scaled up to a large amount of drones/UAVs, collision rates would skyrocket due to the late response. 
- In theory, if we can use the Octree data structure for this then we can reduce the query time multiple times over and drastically reduce the collision percentage.

## Data
- There are two main datasets we've used:
    + https://www.kaggle.com/datasets/ziya07/uav-coordination-dataset/data saved as uav_data.csv used to get the drone coordinates, velocity, yaw, and pitch for the 3D Drone simulation and algorithm benchmarking.
    + The second one is in the Octree_Collision_Warning folder is self generated using the data_generate.cpp file and used for collision warnings with our own Octree implementation in CPP.
 
  - We've processed uav_data.csv file as follow:
      + Standardised the data to coordinates in the simulation space.
      + Calculated movement vectors from the velocity, yaw, and pitch values.
    
## Algorithm hypotheses for avoiding collisions
- The Octree structure space will be divided into 8 equal parts for each subdivision, until the number of drones in a cube is smaller than the allowed amounts.
- To check whether a drone would collide with one another, our program will query the drones in an Octree node with a safety "barrier" around it to check whether one drone's barrier collide with another's. This will help having to query drones far away from each other to check for collisions.
- When two drone's safety barriers collide or their movement vectors suggest that they will collide, the force needed to alter the drones' paths will be calculated to change their flight path:  
        
        if d < radius:
            if d < collision_rad:
                collisions_count += 1
            force = (diff / (d + 1e-9)) * (radius - d)
            repulsion[i] += force
            repulsion[j] -= force

## Implementation
- To run the collision warning system:
  + Run the data_generate.cpp file to generate the data needed. The number of drones can be changed through modifying the config in the main() function.
  + We can now run the Final_Warning_System.cpp to get our results:

 - To run 3D drone simulations and benchmarks:
 - <img width="551" height="241" alt="sample_output" src="https://github.com/user-attachments/assets/0da518a7-f3f0-4d1f-868d-551eb6217738" />

 - Create a venv:
 -     python -m venv venv
 - Activate it:
 -     venv\Scripts\activate
 - Install the dependencies:
 -     pip install -r requirements.txt
 - We can now run the final_visualizer.py file for simulations. Select 1 or 2 for simulating brute force and Octree algorithms:
 -     python final_visualizer.py
 - For our simulations we can tweak the following parameters in the final_visualizer.py file:
    + SPEED_SCALE: Tweak drone speed.
    + DODGE_RADIUS: Tweak the radius where warnings will be given for drone collisions.
    + COLLISION_RADIUS: Tweak the radius of the safety barrier that count drones as collided.
    + ARRIVAL_DIST: Tweak the distance the drone need to reach.
    + MISSION_DIST: Tweak the original distance between the drone and the target.
    + WORLD_SIZE: Tweak the world size of the simulation.
    + OCTREE_CAPACITY: Tweak the maximum amount of drones in one node.
- In our 3D drone simulation we can see:
  + Little yellow transparent dots as the destination that the drones need to reach.
  + Drones are colored as cyan, green, and red dots. Cyan means that they are traveling, green means that they've reached their destination, and red means that they've either collided or is in the process of dodging.
  + During the simulation we can see parameters such as the total dodge count, active drones, build, queries,...
  <img width="726" height="507" alt="sample_visualization" src="https://github.com/user-attachments/assets/59f437f6-e9cb-437a-832d-74b74abbfd3c" />
  (Image for reference)
- We can also run the 3rd option to run pure benchmarks without opening the 3D simulation. There are two options to select from. The Standard Benchmark which will simply run a simulation with X amounts of drones, and the varied benchmarks which will test more aspects of each algorithms.
- Charts will be generated in the benchmark_results folder.
<img width="800" height="600" alt="scaling_curve" src="https://github.com/user-attachments/assets/4dfdce7e-b3a0-499d-a67f-af23b1fc9e43" />
(Example chart)
## Benchmarks, comparisons conclusions
- What is the conclusion?
- Optional


## Reference
- https://www.kaggle.com/datasets/ziya07/uav-coordination-dataset/data
      For the dataset used for this project.
- https://docs.pyvista.org/index.html
     For the drone visualization.
- https://math.stackexchange.com/questions/2618527/converting-from-yaw-pitch-roll-to-vector
     For the formula to get direction vector for updating the drone coordinates every frame.
- https://youtu.be/vwUNtmhPVE0?si=3Jh-DMby4HXXwU_H
     For the dodging mechanisms.
- https://editor.p5js.org/curiouser.kate/full/TJTG-mKVP
     For the basic Octree visualization.
- Claude and Gemini for carrying our group in the process of building the visualization and helping in understanding the Octree algorithm.

## Other information


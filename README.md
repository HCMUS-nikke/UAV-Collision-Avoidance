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

## Benchmarks, comparisons, and conclusions

<img width="800" height="600" alt="scaling_curve" src="https://github.com/user-attachments/assets/03a2b441-f100-43a7-ae44-4a7b17c20910" />

<img width="800" height="600" alt="scaling_curve_crossover" src="https://github.com/user-attachments/assets/115a4717-0ad2-41b9-a360-a0c33ba38ffe" />


Looking at the two charts above. In the first chart, since Brute Force has a complexity of O(n^2), its growth rate is much faster than the Octree algorithm, which only has a complexity of O(n log n). However, for a smaller number of drones, around 32 or fewer, Brute Force is actually faster than Octree. 

This is because Octree requires time and overhead to initialize the tree. Even though this initialization only takes a few milliseconds, it makes up a larger portion of the total time when dealing with a small number of drones. 

Therefore, calculating directly with the Brute Force method is noticeably faster for smaller drone counts since it skips this initialization step. Once the number of drones passes this mark, Octree becomes a lot faster.

<img width="800" height="600" alt="capacity_time_impact" src="https://github.com/user-attachments/assets/927e8713-3e08-4b02-934b-4d3ad93ffd59" />


An important finding the we found out is that the optimization of node capacity (the maximum capacity of each cube) before it subdivides. Based on the graph, if the node capacity is too small, the tree will branch out too deeply, which will have higher memory usage and longer traversal times. If the capacity is too large, it results in too many checks, making it essentially no different from Brute Force. 

After testing, the optimal node capacity was found to be between 6 and 12, depending on the number of drones. These nodes are small enough to filter out drones that are on the opposite side or too far from the calculation area, yet large enough so that the Octree algorithm does not have to create additional branches for nearby drones.

<img width="800" height="600" alt="clustering_penalty" src="https://github.com/user-attachments/assets/4e2a97ac-9ba4-4d22-a17b-17c83ad19164" />


In addition, we also did a stress tests. When the drones are not evenly distributed but instead clustered in one place, the tree becomes unbalanced. This causes the processing time to increase significantly.

<img width="800" height="600" alt="scaling_curve_crossover" src="https://github.com/user-attachments/assets/3da34de8-6817-4c4b-b3c0-f0e7a64ac8c8" />

Furthermore, when measuring the number of collisions, both the Brute Force and Octree algorithms recorded a total of zero collisions during the scaling test from 0 to 400 drones. The overlap of these two graph lines proves that even though the Octree algorithm divides the operating space into smaller areas to reduce the number of calculations, it still ensures absolute accuracy. It does not miss any potential collisions or produce skewed results compared to scanning every single pair of elements in the Brute Force method. 

This confirms that Octree can optimize runtime performance without sacrificing the reliability of the warning system.

Through benchmarking and analyzing the charts, we have concluded both the effectiveness and the weaknesses of the Octree algorithm.

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


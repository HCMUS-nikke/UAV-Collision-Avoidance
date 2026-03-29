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
    
## Algorithm Hypotheses

## Implementation
- Briefly describe the implemetation.
    - How to set up the enviroment to run the source code and required steps to replicate the results
    - Discuss the concrete implementation if there are any essential details
    - How to run each step from `In-sample Backtesting`, Step 4 to `Out-of-sample Backtesting`, Step 6 (or `Paper Trading`, Step 7).
    - How to change the algorithm configurations for different run.
- Most important section and need the most details to correctly replicate the results.

## Optimization

## Comparison

## Benchmarks and conclusions
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


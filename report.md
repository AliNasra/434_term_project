
## Brief Intro:

The final project of CMPE 434 aims to combine the aggregate knowledge accumulated throughout the semester in order to realize an autonomous navigation system for a robot.

## Architecture of the implementation

### Level 1:

This compartment relies on DWA, Dynamic-Window Approach, for guiding the robot through a dungeon-like surrounding beset with dynamic and static obstacles. There are 3 parameters that dictate the choice of candidate DWA-generated trajectory: collision cost, goal cost, and velocity cost. The filtering of trajectories starts by eliminating all the trajectories that lead to a collision. Subsequently, the remaining trajectories are sorted by their velocity with only the top 30% picked. Eventually, the trajectory that aligns the most with the goal is picked.

### Level 2:

To make up for the drawbacks of DWA in closed environments, we generate a global route between the start and end points using **the A\* Method**.  This allows the robot to track local goals sequentially while avoiding being trapped in particular positions within the dungeon maze.

### Communication:

The communication among the architecture's levels occurs within the main.py file. This is an application of the shared memory approach.

### Visualization:
The itinerary of the robot, the pre-planned route, and the selected DWA-generated trajectory are visualized to give the spectators a glimpse of the intense computational activity in the background.

## Observations:
After an intense and protracted period of testing, the following observations were discerned.
1- The offline path planning is immensely time-consuming. Although I tried the configuration recommended by my colleagues, it didn't improve the performance considerably. Subsequently, I concluded that the device's hardware specifications should be taken into consideration during the implementation.
2- The vehicle is highly unstable at hard turns. I attempted to solve this issue by limiting the scope of the sampling pool. It helped to a certain degree.
3- There are situations where all the trajectories are rendered unfeasible. Unfortunately, I couldn't implement a bailout policy for salvaging the robot.
4- Python is very slow when it comes to intensive computational tasks. Libraries like numpy are indispensable in the implementation.
## Figures and illustrations:
### An example of the route and obstacle layout:
![Screenshot from 2024-04-30 14-01-00](https://github.com/AliNasra/434_term_project/assets/52269552/e3f5d3f0-152c-4490-b816-7fc5d99e931d)
### The vehicle avoids an incoming obstacle:
![Screenshot from 2024-05-15 10-25-29](https://github.com/AliNasra/434_term_project/assets/52269552/179df29d-89b7-4ee2-82d4-e59b9293240d)
### The vehicle ramming into the incoming obstacle "Notice that there are no valid trajectories":
![Screenshot from 2024-05-15 10-07-50](https://github.com/AliNasra/434_term_project/assets/52269552/fc24b292-2d91-43e3-98a4-63e7abdc5858)
### The vehicle reaching the end goal:
![Screenshot from 2024-05-13 10-50-58](https://github.com/AliNasra/434_term_project/assets/52269552/22824021-8509-492e-bea8-4c8d6c8c6fc8)



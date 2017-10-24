# Intersection Navigation using RRT

Solving Intersection Navigation for a car using Rapidly Exploring Random Tree (RRT) algorithm under dynamic constraints. This was implemented as a part of solution to assignment-3 of CS-6244 taught by Prof. David Hsu.

## Getting Started

There are two implementations of the algorithm. One computes the plan offline and other computes it online.
To run the code, it will need an input file and a output directory where it will save the controls computed by the algorithm. More on the syntax of the files in later sections

### Prerequisites

You will need python 3 to run the scripts, along with numpy package.

## Algorithm

The algorithm first selects a node, m_g, to expand via random sampling from given range (note: range changes as we reach a goal path to further concentrate on finding a path with shorter total time) and explore in the free space (like we do in RRT). After m_g has been selected, it selects a random value of acceleration to apply on the state m_g with some bias to direct the graph towards the final position. As *dt* is assumed to be 0.1 second, we calculate the new state, m_new, by integrating value of acceleration over *dt*. The trajectory from m_g to m_new, is then checked for collisions. If trajectory is collision free, m_new is added to the graph and corresponding control is stored with edge (m_g, m_new). After the limit on number of samples is reached, we get a graph with multiple trajectories from initial position to goal position. We then select the trajectory which took the shortest time and save it.

## Design choices

- **Selection of node to expand**: One of the key element was to design selection of node to expand. Uniform selection of nodes in the graph would have resulted in overloading of an area with many nodes and not much free area would have been explored. So, some kind of bias was needed to direct the exploration away from the current density. We could use EST like weighted sampling by assigning each node with a weight inversely proportional to its density as done in [Hsu02]. But I found that calculating spatial density for each node was computational expensive. Alternatively, I could have divided nodes into bins and then choose a bin uniformly and then pick a node from selected bin. It would have also resulted in better exploration than standard uniform selection of nodes. Another way was to give weight inversely proportional to degree of the node, but this resulted in requiring exponential number of nodes to reach target position (very bad). I found implementing RRT style selection of a graph node easy and not much computationally expensive. I controlled the range of sampling of random point (by controlling the value of ) to direct the graph to a particular region of interest. Particularly, once the algorithm finds a path from initial to final position, we want to focus the scope of expansion to nodes which occurred prior to time taken by final position to get a better path taking lesser time.
- **Checking collision**: It was also important piece of the algorithm. To check for collision, I expanded the size of obstacles and reduced the size of our robot to point object. As this is a car and we care for it, I further added some padding to this increased obstacle size to prevent cases of near miss and have some buffer space. But my work did not end here as there was one more problem to be solved. We can reach a position  at time  with multiple possible velocity. Some of them might be so high that even if we apply maximum deceleration, we could not prevent it from crashing in (call this area as area of imminent collision). So, every obstacle vehicle was given with extra padding, given by velocity and maximum acceleration allowed (e.g. we will need a padding of 1.25 if v=5 and a=-10. In this way, we are better able to sample those points which are feasible. Further, I had to control the size of padding, because as I increased the padding, completeness of my algorithm decreased.
- **Number of Samples**: Running time of my algorithm was directly impacted by number of samples drawn. As number of samples increased, completeness and optimality of the algorithm increased. Due to much more samples available, algorithm could find a path if it exists with more probability of success or could focus more on finding a better path than current best. I used around 5,000 samples per data and it returned the shortest possible path almost every time.

### Input file: future positions of the agent cars

Description of input file: the input file provides the future positions of other agent cars. Algorithm will need to query the position of 'robot_i' at time step t_i to plan controls for the autonomous car. We assume that the simulation starts at t_0 = 0.

File format of dataXX.json:

{'robot_1': [[x_0, y_0, ori_0 t_0], [x_1, y_1, ori_1, t_1], ... [x_N, y_N, ori_N, t_N]]

. . .

'robot_K': [[x_0, y_0, ori_0, t_0], [x_1, y_1, ori_1, t_1], ... [x_N, y_N, ori_N, t_N]] }

where K is the number of agent cars on road. N is the length of the recorded data sequence. x_i, y_i, ori_i (i = 0, 1, ... , N) are the x-coordinate, y-coordiate and orientation at the ith time step. t_0, ... , t_N are the timestamps of the data sequence, and the simulation starts at t_0 = 0.

### Output file: the controls computed by motion planner

Output file: 'dataXX_controls.json'

Description of output file: the purpose of the output file is to record the planned controls for the autonomous car.

File format of dataXX_controls.json:

{'robot_0': [[acc_1, t_1], ... [acc_M, t_M]] }

where M is the number of steps it takes for the autonomous car to cross the road. acc_i is the acceleration of the autonomous car at timestamp t_i. Note that for simplicity, we assume the control duration at each step is 0.1s, i.e. t_i = i * 0.1

## Authors

* **Dixant Mittal** - [dixantmittal](https://github.com/dixantmittal)

## License

This project is licensed under the Apache License - see the [LICENSE](LICENSE) file for details

## References

* [Hsu02] - Hsu, D., Kindel, R., Latombe, J.-C., & Rock, S. (2002). Randomized Kinodynamic Motion Planning with Moving Obstacles. The International Journal of Robotics Research, 21(3), 233–255. https://doi.org/10.1177/027836402320556421

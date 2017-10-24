# Intersection Navigation using RRT

Solving Intersection Navigation for a car using Rapidly Exploring Random Tree (RRT) algorithm under dynamic constraints. This was implemented as a part of solution to assignment-3 of CS-6244 taught by Prof. David Hsu.

## Getting Started

There are two implementations of the algorithm. One computes the plan offline and other computes it online.
To run the code, it will need an input file and a output directory where it will save the controls computed by the algorithm. To read more about the algorithms, read the corresponding pdfs. More on the syntax of the files in later sections.

### Prerequisites

You will need python 3 to run the scripts, along with numpy package.

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

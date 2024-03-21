# Maximizing Safety and Efficiency for Cooperative Lane-Changing: A Minimally Disruptive Approach

[<img src="https://img.shields.io/badge/arxiv-%23B31B1B.svg?&style=for-the-badge&logo=arxiv&logoColor=white" />](https://arxiv.org/abs/2305.17883)

[<img src="https://www.vectorlogo.zone/logos/ieee/ieee-ar21.svg">](https://ieeexplore.ieee.org/document/10422143)


## Overview

This repository contains code to accompany the paper "Maximizing Safety and Efficiency for Cooperative Lane-Changing: A Minimally Disruptive Approach" by Armijos, Li, and Cassandras. 

The paper addresses cooperative lane-changing maneuvers in mixed traffic, aiming to minimize traffic flow disruptions while accounting for uncooperative vehicles. The proposed approach uses Optimal Control Barrier Functions (OCBF) to guarantee spatio-temporal constraints and introduce robustness to disturbances.

## Requirements

To run the code, you will need:

- CASADI
- MATLAB 2021b or later with the Autonomous Driving Toolbox

## Usage

To run the code, simply execute the `main.m` script in MATLAB. 

The `main.m` script will run the simulations and generate the results.

## Paper abstract

This paper addresses cooperative lane-changing maneuvers in mixed traffic, aiming to minimize traffic flow disruptions while accounting for uncooperative vehicles. The proposed approach adopts controllers combining Optimal control with Control Barrier Functions (OCBF controllers) which guarantee spatio-temporal constraints through the use of fixed-time convergence. Additionally, we introduce robustness to disturbances by deriving a method for handling worst-case disturbances using the dual of a linear programming problem. We present a near-optimal solution that ensures safety, optimality, and robustness to changing behavior of uncooperative vehicles. Simulations demonstrate the effectiveness of the proposed approach in enhancing efficiency and safety.


## Citing this work


```bibtex
@INPROCEEDINGS{10422143,
  author={Chavez Armijos, Andres S. and Li, Anni and Cassandras, Christos G.},
  booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={Maximizing Safety and Efficiency for Cooperative Lane-Changing: A Minimally Disruptive Approach}, 
  year={2023},
  volume={},
  number={},
  pages={4272-4277},
  keywords={Optimal control;Linear programming;Robustness;Safety;Behavioral sciences;Intelligent transportation systems;Convergence},
  doi={10.1109/ITSC57777.2023.10422143}}
}
```




## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.

# Maximizing Safety and Efficiency for Cooperative Lane-Changing: A Minimally Disruptive Approach

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

## Reference
@article{
author={Armijos, A. S. C. and Li, A. and Cassandras, C. G.},
title={Maximizing Safety and Efficiency for Cooperative Lane-Changing: A Minimally Disruptive Approach},
journal={26th IEEE Conf. on Intelligent Transportation Systems ITSC},
year={2023}
}


## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.

Let me know if you would like me to modify or add anything to this README!

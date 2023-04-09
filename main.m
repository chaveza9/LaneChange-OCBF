clear all
close all
clc
if ismac
    addpath('./casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('.\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*

gen_plot = true;
% Initial Conditions
x0 = [-2.0, 0.0, 0.0, 0.0]';
% Define problem
x_goal = [1, 0.001, 0.5, 0.0];
x_obst = [0, 0];
r = 0.5;
% Time parameters
dt = 0.1; % step size [s]
H = 3; % horizon time [s]
substeps = 10;
T  = 10; % terminal time [s]
% Define matrix values for states
Q = diag([0.001, 1.0, 0.1, 0.0]);
R = 0.01 * eye(2);
% Define Control limits
cntrl_lims = [1, 0.5];
% Define speed lims
speed_lims = [0.01, 2];

% Define agent
agent = DubinsAgent(dt, H, substeps, x_goal, r, Q, R, speed_lims, cntrl_lims);

% Simulate Agent
[x_hist, u_hist]= agent.simulate( T, x0, x_obst, gen_plot);


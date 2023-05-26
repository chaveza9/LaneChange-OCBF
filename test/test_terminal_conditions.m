clc
clear
close all

import casadi.*

%% Environmental Setting
% Simulation setting
StopTime = 10;
dt = 0.01;
% CAV set
num_vehicles = 5; % number of vehicles in fast lane
v_des_range = [20,33];
min_pos = 0;
% Maneuver constraints
v_des = 29; %m/s
reactionTime = 0.9;
minSafeDistance = 15;
% Vehicle constraints
constraints.u_max = 3.3;     % Vehicle i max acceleration
constraints.u_min = -7;      % Vehicle i min acceleration
constraints.v_max = 35;      % Vehicle i max velocity
constraints.v_min = 10;      % Vehicle i min velocity
%% Initial Conditions
% Preallocate vehicle array
X_0 = zeros(5,2);
states.Position = [0,0]';
states.Velocity = 0;
states.Heading = 0;
states_cav = repelem(states, num_vehicles);
% Vehicle U initial States;
states_u = states;
states_u.Position(1) = 100;
states_u.Velocity = 20;
% Vehicle C States;
states_c = states;
states_c.Position(1) = 30;
states_c.Velocity = 24;
% Vehicle 1 States;
states_cav(1).Position(1) = 115;
states_cav(1).Velocity =  25 + rand*(10);
% Vehicle 2 States;
states_cav(2).Position(1) = 85;
states_cav(2).Velocity = 25 + rand*(10);
% Vehicle 3 States;
states_cav(3).Position(1) = 60;
states_cav(3).Velocity = 25 + rand*(10);
% Vehicle 4 States;
states_cav(4).Position(1) = 25;
states_cav(4).Velocity = 25 + rand*(10);
% Vehicle 5 States;
states_cav(5).Position(1) = 0;
states_cav(5).Velocity = 15 + rand*(10);

% Solve  MINLP
[tf, x_e_f, v_e_f, x_f, v_f, B, i_m] = ...
Collab.define_terminal_conditions (states_c, states_cav, states_u, ...
    constraints, v_des, reactionTime, minSafeDistance);

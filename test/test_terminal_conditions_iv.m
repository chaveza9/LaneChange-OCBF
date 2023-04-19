clear all
close all
clc
% add source library is in the path
addpath(['..',filesep,'src',filesep]);
import casadi.*
tic
%% Environmental Setting
% Simulation setting
StopTime = 10;
dt = 0.01;
% CAV set
num_vehicles = 5; % number of vehicles in fast lane
v_des_range = [25,30];
min_pos = 0;
% Maneuver constraints
v_des = 30; %m/s
reactionTime = 0.9;
minSafeDistance = 15;
% Vehicle dimensions
params.actors.carLen   = 4.7; % [m]
params.actors.carWidth = 1.8; % [m]
params.actors.rearAxleRatio = .25;
% Define road dimensions
params.road.laneWidth   = 3.6; % [m] 
params.road.length = 3000; %[m]
% Vehicle constraints
constraints.u_max = 3.3;     % Vehicle i max acceleration
constraints.u_min = -3;      % Vehicle i min acceleration
constraints.v_max = 35;      % Vehicle i max velocity
constraints.v_min = 10;      % Vehicle i min velocity
%% Initial Conditions
% Preallocate vehicle array
states.Position = [0,params.road.laneWidth*0.5]';
states.Velocity = 0;
states.Heading = 0;
states_cav = repelem(states, num_vehicles);
% Vehicle U initial States;
states_u = states;
states_u.Position(1) = 100;
states_u.Position(2) = -params.road.laneWidth*0.5;
states_u.Velocity = 20;
% Vehicle C States;
states_c = states;
states_c.Position(1) = 30;
states_c.Position(2) = -params.road.laneWidth*0.5;
states_c.Velocity = 24;
% Vehicle 1 States;
states_cav(1).Position(1) = 115;
states_cav(1).Velocity = 28;
% Vehicle 2 States;
states_cav(2).Position(1) = 85;
states_cav(2).Velocity = 28;
% Vehicle 3 States;
states_cav(3).Position(1) = 60;
states_cav(3).Velocity = 24;
% Vehicle 4 States;
states_cav(4).Position(1) = 25;
states_cav(4).Velocity = 24;
% Vehicle 5 States;
states_cav(5).Position(1) = 0;
states_cav(5).Velocity = 24;
%% Create Scenario and initialize vehicles
% Create a driving scenario
scenario = Env.ds4vehicleScenario(params);
% Create CAV C
cav_c = IntelligentVehicle('c', scenario, states_c, StopTime, ...
    constraints, 'SafetyDistance',minSafeDistance, 'SampleTime', dt, ...
    'ReactionTime',reactionTime); 
% Create Vehicle U
veh_u = IntelligentVehicle('u', scenario, states_u, StopTime,  ...
    constraints, 'SafetyDistance',minSafeDistance, 'SampleTime', dt, ...
    'ReactionTime',reactionTime, 'VehicleType','NonControlled', ...
    'VehicleClass',2); 
% Create CAV Set
cav_set = repelem(cav_c, num_vehicles, 1);
for i=1:num_vehicles
    vehID = num2str(i);
    cav_set(i) =  IntelligentVehicle(vehID, scenario, states_cav(i), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime);  
end
fprintf("Has Created Scenarios and Vehicles, elapsed time: %f \n", toc)
% Extended cav_set 
cav_env = [cav_set;cav_c];
%% Compute terminal conditions
fprintf("Solving Terminal Conditions ...\n")
tic
[tf, x_e_f, v_e_f, x_f, v_f, B, i_m] = ...
Collab.define_terminal_conditions (states_c, states_cav, states_u, ...
    constraints, v_des, reactionTime, minSafeDistance);
fprintf("Has solved terminal conditions, elapsed time: %f \n", toc)

%% Compute OCP for each vehicle
fprintf("Solving analytical OCP ...\n")
tic
% Extract optimal collaborative vehicles ids
veh_c_id = 'c';
veh_1_id = num2str(i_m-1);
veh_2_id = num2str(i_m);
% cav c
hasDefinedRoll = cav_c.define_cav_roll("cavC", tf, x_e_f, v_des,...
    cav_env, 'f_collab_id',veh_1_id, 'r_collab_id',veh_2_id);
for i=1:num_vehicles

    if i ~= i_m % Accelerating Vehicles
        hasDefinedRoll = cav_set(i).define_cav_roll("cav1", tf, ...
            x_f(i), v_f(i), cav_env);
    else % Decelerating Vehicle
        hasDefinedRoll = cav_set(i).define_cav_roll("cav2", tf, x_f(i), ...
            v_f(i), cav_env,...
            'f_collab_id',veh_1_id, 'e_collab_id',veh_c_id);
    end
    if ~hasDefinedRoll
        warning('solution not found, for cav %s', cav_set(i).VehicleID)
    end
end
fprintf("Has solved computed ocp, elapsed time: %f \n", toc)



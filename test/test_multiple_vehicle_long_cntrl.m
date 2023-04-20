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
% Create CAV Set
cav_set = [];
for i=1:num_vehicles
    vehID = num2str(i);
    veh = IntelligentVehicle(vehID, scenario, states_cav(i), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime);  
    cav_set = cat(1,cav_set,veh);
end
% Create Chase plot 
figScene = Env. createVisualizationPlot(scenario,params,num2str(num_vehicles));
%% Define terminal conditions
tf = 7.38;
x_f = [362.7484,322.5495,284.7445,247.4885, 178.4438]';
v_f = [28.0000,27.9988,25.3389,24.7289,21.5156]';
% Compute Analytical OCP 
%% Compute OCP
for i=1:num_vehicles
    hasDefinedRoll = cav_set(i).define_cav_roll("cav1", tf, ...
        x_f(i), v_f(i), cav_set);
    if ~hasDefinedRoll
        warning('solution not found, for cav %s', cav_set(i).VehicleID)
    end
end
%% Step Through cav
for t = 0:dt:tf+dt
    % Compute CBF
    for i=1:num_vehicles
        status = cav_set(i).step;
        if ~status
            disp(t)
            warning('invalid solution')
            break
        end
    end
    % Advance simulation
    advance(scenario)
end
% Display the terminal position for each cav
    ter_pos = arrayfun(@(x) x.CurrentState.Position',cav_set,'UniformOutput',false);
disp(ter_pos);

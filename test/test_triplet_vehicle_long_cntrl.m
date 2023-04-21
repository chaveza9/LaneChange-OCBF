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
num_vehicles = 2; % number of vehicles in fast lane
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
% Vehicle C States;
states_c = states;
states_c.Position(1) = 30;
states_c.Position(2) = -params.road.laneWidth*0.5;
states_c.Velocity = 24;
% Vehicle 1 States;
states_cav(1).Position(1) = 25;
states_cav(1).Velocity = 24;
% Vehicle 2 States;
states_cav(2).Position(1) = 0;
states_cav(2).Velocity = 24;
%% Create Scenario and initialize vehicles
% Create a driving scenario
scenario = Env.ds4vehicleScenario(params);
% Create CAV C
cav_c = IntelligentVehicle('c', scenario, states_c, StopTime, ...
    constraints, 'SafetyDistance',minSafeDistance, 'SampleTime', dt, ...
    'ReactionTime',reactionTime); 
% Create CAV Set
cav_set = [];
for i=1:num_vehicles
    vehID = num2str(i);
    veh = IntelligentVehicle(vehID, scenario, states_cav(i), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime);  
    cav_set = cat(1,cav_set,veh);
end
cav_set = [cav_set;cav_c];
% Create Chase plot 
figScene = Env. createVisualizationPlot(scenario,params,num2str(num_vehicles));
%% Define terminal conditions
% terminal time
tf = 7.38;
% Vehicle C terminal conditions
x_e_f = 212.8078;
v_e_f = 21.8674;
% CAV Set terminal conditions
x_f = [247.4885, 178.4438, x_e_f]';
v_f = [24.7289,21.5156,v_e_f]';
% Compute Analytical OCP 
%% Compute OCP
veh_c_id = 'c';
veh_1_id = '1';
veh_2_id = '2';
cav_c.define_cav_roll("cavC", tf, x_e_f, v_des,cav_set, ...
    'f_collab_id',veh_1_id, 'r_collab_id',veh_2_id);
cav_set(1).define_cav_roll("cav1", tf, x_f(1), v_des,cav_set);
hasDefinedRoll = cav_set(2).define_cav_roll("cav2", tf, x_f(i), v_f(i),...
    cav_set,'f_collab_id',veh_1_id, 'e_collab_id',veh_c_id);

%% Step Through cav
for t = 0:dt:tf           
    % Compute CBF
    for i=1:num_vehicles+1
        status = cav_set(i).step;
        if ~status
            disp(t)
            warning('invalid solution')
            break
        end
    end
    % Advance simulation
    advance(scenario);
end
% Display the terminal position for each cav
    ter_pos = arrayfun(@(x) x.CurrentState.Position',cav_set,'UniformOutput',false);
disp(ter_pos);

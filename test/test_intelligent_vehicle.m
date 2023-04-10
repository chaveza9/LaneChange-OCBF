clear all
close all
clc
% add source library is in the path
addpath(['..',filesep,'src',filesep]);
% Vehicle dimensions
params.actors.carLen   = 4.7; % [m]
params.actors.carWidth = 1.8; % [m]
params.actors.rearAxleRatio = .25;
% Define road dimensions
params.road.laneWidth   = 3.6; % [m] 
params.road.length = 3000; %[m]
% Vehicle constraints
constraints.u_max = 3.3;     % Vehicle i max acceleration
constraints.u_min = -7;      % Vehicle i min acceleration
constraints.v_min = 10;      % Vehicle i min velocity
constraints.v_max = 35;      % Vehicle i max velocity
% Initial Conditions
x_0_c.Position = [10, 0]';
x_0_c.Velocity = 10;
x_0_c.Heading = 0;

VehID = '1';

% Maneuver 
StopTime = 10;
t_f = 10;
v_des = 30;
x_f = 20;

% Create a driving scenario
scenario = Env.ds4vehicleScenario(params);

% Create Vehicle
cav = IntelligentVehicle(VehID, scenario, x_0_c, StopTime, constraints); 

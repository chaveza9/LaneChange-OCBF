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
% Vehicle C
x_0_c.Position = [30, 0]';
x_0_c.Velocity = 24;
x_0_c.Heading = 0;


VehID = '1';

% Maneuver 
StopTime = 10;
tf = 1.7126*1;
v_des = 28;
x_f = 45.9415;
tf = 7.3744;
x_f = 322.5495;
x_0_c.Position(1) = 85;
x_0_c.Velocity = 28;

% Create a driving scenario
scenario = Env.ds4vehicleScenario(params);

% Create Vehicle
cav = IntelligentVehicle(VehID, scenario, x_0_c, StopTime, constraints); 
% Compute Analytical OCP 
% hasDefinedRoll = cav.define_cav_roll("cav1", tf, x_f, v_des);

[tf,x_C_f,v_C_f,posStates,speedStates,accTraj,cost] = solve_v_des(x_0_c.Position(1),x_0_c.Velocity,115,28,250,tf, x_f);

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
constraints.u_min = -3;      % Vehicle i min acceleration
constraints.v_min = 10;      % Vehicle i min velocity
constraints.v_max = 35;      % Vehicle i max velocity
% Initial Conditions
% Vehicle C



VehID = '1';

% Maneuver 
dt = 0.1;
StopTime = 10;
tf = 7.3744;
x_f = 362.7484;
v_des = 28;
x_0_c.Position = [115, 0]';
x_0_c.Velocity = 28;
x_0_c.Heading = 0;

% Create a driving scenario
scenario = Env.ds4vehicleScenario(params);

% Create Vehicle
cav = IntelligentVehicle(VehID, scenario, x_0_c, StopTime, constraints,...
    'SampleTime', dt);
% Compute Analytical OCP 
hasDefinedRoll = cav.define_cav_roll("cav1", tf, x_f, v_des, "verbose",1);
% Step Through cav
for t = 0:dt:StopTime
    % Compute CBF
    status = cav.step();
    if ~status
        disp(t)
        warning('invalid solution')
        break
    end
    % Advance simulation
    advance(scenario)
end


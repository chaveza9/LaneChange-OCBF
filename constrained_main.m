clear all
close all
clc
% add source library is in the path
addpath(['.',filesep,'src',filesep]);
import casadi.*
tic
%% Environmental Setting
StoreResults = 1;
TOD = datetime('now','TimeZone','local','Format','MM-dd-yyyy_HH-mm');
TOD = string(TOD);
% Simulation setting
StopTime = 13;
dt = 0.05;
% CAV set
num_vehicles = 5; % number of vehicles in fast lane
v_des_range = [25,30];
min_pos = 0;
% Maneuver constraints
v_des = 34; %m/s
reactionTime = 0.8;
minSafeDistance = 7;
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
constraints.v_max = 35;      % Vehicle i max velocity
constraints.v_min = 10;      % Vehicle i min velocity
%% Initial Conditions
% Preallocate vehicle array
states.Position = [0,params.road.laneWidth*0.5]';
states.Velocity = 0;
states.Heading = 0;
states.Steering = 0;
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
cav_set(1) =  IntelligentVehicle('F', scenario, states_cav(1), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime, ...
        'VehicleType','NonControlled', 'VehicleClass',2); 
for i=2:num_vehicles
    vehID = num2str(i);
    cav_set(i) =  IntelligentVehicle(vehID, scenario, states_cav(i), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime);  
end
fprintf("Has Created Scenarios and Vehicles, elapsed time: %f \n", toc)
% Extended cav_set 
cav_env = [cav_set;cav_c;veh_u];
% Create Chase plot 
figScene = Env. createVisualizationPlot(scenario,params,num2str(num_vehicles));
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
for i=2:num_vehicles
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

%% Step Through cav
fprintf("Stepping though...\n")
compute_time = [];
frameCount = 1;
StopTime = tf;
for t = 0:dt:StopTime + 1.5        
    % Compute CBF
    tic
    for i=1:num_vehicles+2
        status = cav_env(i).step;
        if ~status
            disp(t)
            warning('invalid solution')
            break
        end
    end
    compute_time = cat(1,compute_time, toc);
    % Advance simulation
    advance(scenario);
    % If video is requested
    if StoreResults
        Frames(frameCount) = getframe(gcf);
        frameCount = frameCount+1;
    end

end
fprintf("Has Finished simulation, average step compute time: %f \n", mean(compute_time))
% Display the terminal position for each cav
ter_pos = cell2mat(arrayfun(@(x) x.CurrentState.Position', ...
    cav_env,'UniformOutput',false));
disp(ter_pos(:,1)-[x_f;x_e_f;0] );

% Store Files

if StoreResults
    % Create containing folder 
    location = strcat('.',filesep,'Results',filesep,TOD);
    status = mkdir(location);
    filename = strcat(location,filesep,'test_',TOD,'.mp4');
    fprintf("Generating video...\n")
    writerObj = VideoWriter(filename,'MPEG-4');
    writerObj.FrameRate = round(frameCount/StopTime);
    open(writerObj)
    writeVideo(writerObj, Frames);
    close(writerObj);
end

% Create figures
if ~StoreResults
    location = [];
end

Utils.plot_state_history(cav_env, tf, i_m, location)

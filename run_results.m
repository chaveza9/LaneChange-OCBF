function [cav_set,cav_c, cav_env]= run_results(fid, options)
arguments
    fid = 0;
    options.method (1,:) char {mustBeMember(options.method,{'fxtm','cbf','ocbf'})} = 'fxtm'
    options.noise logical = 0
    options.num_uncooperative (1,:) double {mustBeNumeric, mustBeNonnegative} = 1;
    options.adaptive_safety logical = 1; 
    options.store_restuls logical = 1;
end
tic
close all
%% Environmental Setting
StoreResults = options.store_restuls;
TOD = datetime('now','TimeZone','local','Format','MM-dd-yyyy_HH-mm');
TOD = string(TOD);
% Simulation setting
StopTime = 14;
dt = 0.05;
% CAV set
num_vehicles = 5; % number of vehicles in fast lane
% v_des_range = [25,30];
% min_pos = 0;
% Maneuver constraints
v_des = 34; %m/s
reactionTime = 0.8;
minSafeDistance = 7;
if options.num_uncooperative>0
    uncooperative_decel = -2.5;
end
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
    'ReactionTime',reactionTime, 'Noise', options.noise, ...
    'Method', options.method, 'AdaptiveSafety', options.adaptive_safety); 
% Create Vehicle U
veh_u = IntelligentVehicle('u', scenario, states_u, StopTime,  ...
    constraints, 'SafetyDistance',minSafeDistance, 'SampleTime', dt, ...
    'ReactionTime',reactionTime, 'VehicleType','NonControlled', ...
    'VehicleClass',2,'UDecel',uncooperative_decel,...
    'Noise', options.noise);
% Create CAV Set
cav_set = repelem(cav_c, num_vehicles, 1);
starting_index = 1;
if options.num_uncooperative>1
    cav_set(1) =  IntelligentVehicle('FU', scenario, states_cav(1), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime, ...
        'VehicleType','NonControlled', 'VehicleClass',2,...
        'UDecel',uncooperative_decel,'Noise', options.noise);
    starting_index = 2;
end
for i=starting_index:num_vehicles
    vehID = num2str(i);
    cav_set(i) =  IntelligentVehicle(vehID, scenario, states_cav(i), ...
        StopTime,  constraints, 'SafetyDistance',minSafeDistance,...
        'SampleTime', dt, 'ReactionTime',reactionTime,...
        'Noise', options.noise, 'Method', options.method,...
        'AdaptiveSafety', options.adaptive_safety);   
end
fprintf("Has Created Scenarios and Vehicles, elapsed time: %f \n", toc)
% Extended cav_set 
cav_env = [cav_set;cav_c;veh_u];
% Create Chase plot 
Env.createVisualizationPlot(scenario,params,num2str(num_vehicles));
%% Compute terminal conditions
fprintf("Solving Terminal Conditions ...\n")
tic
[tf, x_e_f, ~, x_f, v_f, ~, i_m] = ...
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
cav_c.define_cav_roll("cavC", tf, x_e_f, v_des,...
    cav_env, 'f_collab_id',veh_1_id, 'r_collab_id',veh_2_id);
for i=starting_index:num_vehicles
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
for t = 0:dt:StopTime %+ 3.5        
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
        Frames(frameCount) = getframe(gcf); %#ok<AGROW,SAGROW,UNRCH>
        frameCount = frameCount+1;
    end
end
fprintf("Has Finished simulation, average step compute time: %f \n", mean(compute_time))
% Store Files
if StoreResults
    Utils.store_results(TOD,frameCount,StopTime, Frames, cav_env,...
    tf, i_m, comment); 
else
    % Create history plots
    Utils.plot_state_history(cav_env, tf, i_m, [])
end
% Compute Statistics
metric = compute_stats(cav_set, cav_c);
% Create report 
fprintf(fid, strcat('%s \t %s \t %d \t %d \t %d \t %.2f \t %.2f \t ',...
                    '%.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t ',...
                    '%.2f \t %.2f \n'),...
             TOD, options.method, options.num_uncooperative, ...
             options.adaptive_safety, options.noise, ...
             mean(metric.dx.ideal), mean(metric.dx.actual),...
             mean(metric.dv.ideal), mean(metric.dv.actual),...
             mean(metric.TotalDis.ideal), mean(metric.TotalDis.actual),...
             mean(metric.Energy.ideal), mean(metric.Energy.actual),...
             mean(metric.Acc_Diff.mean), mean(metric.Acc_Diff.std));
end

function metric = compute_stats(cav_set, cav_c)
    % Extract longitudinal maneuver portion
    t0_lc = cav_c.LaneChangeTime_start;
    % Create placeholders
    metric.dx.ideal = zeros(1,length(cav_set)+1);
    metric.dx.actual = zeros(1,length(cav_set)+1);
    metric.dv.ideal = zeros(1,length(cav_set)+1);
    metric.dv.actual = zeros(1,length(cav_set)+1);
    metric.TotalDis.ideal = zeros(1,length(cav_set)+1);
    metric.TotalDis.actual = zeros(1,length(cav_set)+1);
    metric.Energy.ideal = zeros(1,length(cav_set)+1);
    metric.Energy.actual = zeros(1,length(cav_set)+1);
    metric.Acc_Diff.mean = zeros(1,length(cav_set)+1);
    metric.Acc_Diff.std = zeros(1,length(cav_set)+1);
    % Append to cav_set
    if contains(cav_set(1).VehicleID,'u','IgnoreCase',true)
        start_index = 2;
    else
        start_index = 1;
    end
    cav_set = [cav_set(start_index:end);cav_c];
    % Extract Vehicle Metrics
    for i=1:length(cav_set)
        [metric.dx.ideal(i), metric.dv.ideal(i), ...
         metric.dx.actual(i), metric.dv.actual(i), ...
         metric.TotalDis.ideal(i), metric.TotalDis.actual(i), ...
         metric.Acc_Diff.mean(i), metric.Acc_Diff.std(i),...
         metric.Energy.ideal(i), metric.Energy.actual(i)] = ...
                cav_set(i).compute_performance_metrics(t0_lc);
    end
    
end



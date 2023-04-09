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
%% Problem Parameters
l = 4; % Lane width [m]
r = 3;  % Vehicle radius [m]
% Define Control limits
cntrl_lims = [3.3, 0.5]; %[u, phi] [m/s^2, rad/s]
% Define speed lims
speed_lims = [13, 30]; %[vmin, vmax]
% Time parameters
dt = 0.1; % step size [s]
H = 5; % horizon time [s]
substeps = 10;
T  = 10; % terminal time [s]
% Solver Paramters
max_iter = 10;
max_tol = 0.5;

%% Agent 1
% Initial Conditions
x_1_0 = [0, l/2, 20.0, 0.0]'; % [x, y, v, theta] [m, m, m/s, rad]
% Define problem
x_1_goal = [1000, -l/2, 25, 0.0]; % [x, y, v, theta] [m, m, m/s, rad] 
% Define matrix values for states
Q = diag([0.5, 1.0, 0.5, 0.0]);
R = 0.01 * eye(2);
% Define Agent
agent1 = DubinsAgent(dt, H, substeps, x_1_goal, r, Q, R, speed_lims, cntrl_lims, false);

%% Agent 2
% Initial Conditions
x_2_0 = [0, -l/2, 20.0, 0.0]'; % [x, y, v, theta] [m, m, m/s, rad]
% Define problem
x_2_goal = [1000, -l/2, 25, 0.0]; % [x, y, v, theta] [m, m, m/s, rad] 
% Define matrix values for states
Q = diag([1, 1.0, 0.5, 0.0]);
R = 0.01 * diag([1,20]);
% Define Agent
agent2 = DubinsAgent(dt, H, substeps, x_2_goal, r, Q, R, speed_lims, cntrl_lims, false);

%% Create simulation 
% Buffer history
x1_hist = [];
u1_hist = [];
x2_hist = [];
u2_hist = [];
% util variables
end_sim = 0; % boolean for end simulation
t = 0; % current sim time
n_infeasible = 0;
x1_guess = nan;
u1_guess = nan;
x2_guess = nan;
u2_guess = nan;
% Define a progress bar
w = waitbar(0,'Initializing problem');
% Initialize ego 2 states
x_obst_1 = zeros(4,H/dt);
u_1_prev = 1000*ones(2,1);
while ~end_sim
    % Append History
    x1_hist= cat(2,x1_hist,x_1_0);
    x2_hist= cat(2,x2_hist,x_2_0);
    for i = 1:max_iter
        % Solve MPC CBF-CLF Agent 2
        [status2, u_2_0, x2_guess, u2_guess] = agent2.ibr_step( x_2_0, x_obst_1,...
            u2_guess, x2_guess);
        % Extract agent 2 as obstacle for agent 1
        x_obst_2 = x2_guess;
        
        % Solve MPC CBF-CLF Agent 1
        [status1, u_1_0, x1_guess, u1_guess] = agent1.ibr_step( x_1_0, x_obst_2,...
            u1_guess, x1_guess);
        % Extract agent 1 as obstacle for agent 2
        x_obst_1 = x1_guess;
        
        % Check if is within tolerace
        if norm(u_1_prev - u_1_0) <= max_tol
            break
        else
            u_1_prev = u_1_0;
        end
    end
    if ~(status1 && status2)
        n_infeasible = n_infeasible +1;  
    end
    % Integrate forward
    t_end_t = min(t + dt, t0+T);
    if t_end_t == t0+T || norm(x_1_0(2)-x_1_goal(2),2)<=0.5
        end_sim = 1;
    end
    t = t_end_t;
    x_1_0 = integrate_forward(t, x_1_0, u_1_0, dt, substep);
    x_2_0 = integrate_forward(t, x_2_0, u_2_0, dt, substep);
    % Append Control History
    u1_hist = cat(2,u1_hist,u_1_0);
    u2_hist = cat(2,u1_hist,u_2_0);
    % update progress bar
    msg = sprintf('Computing MPC solution at time %.2f s/%.2f s  \n',t,T);
    waitbar(t/T, w, msg);
end

% Close window
close(w)

% Plot trajectories
figure
fig = figure;
plot(x1_hist(1, :), x1_hist(2, :)); hold on;
plot(x2_hist(1, :), x2_hist(2, :)); 
legend('Veh1', 'Veh2')
grid minor
lim_min_x = min(min(x1_hist(1, :)), min(x2_hist(1, :)));
lim_max_x = max(max(x1_hist(1, :)), max(x2_hist(1, :)));
lim_min_y = min(min(x1_hist(2, :)), min(x2_hist(2, :)));
lim_max_y = max(max(x1_hist(2, :)), max(x2_hist(2, :)));
xlim([lim_min_x, lim_max_x]);
ylim([lim_min_y, lim_max_y]);
xlabel('x [m]')
ylabel('y [m]')
grid minor



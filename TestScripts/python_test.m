clear all
close all
clc
if ismac
    addpath('./casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('.\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*


% Initial Conditions
x0 = [5.0, 25.0, 0.0, 0.0]';
% Buffer history
x_history = [];
t = 0;
dt = 0.1;
t0 = 0;
T  = 100;
end_sim = 0;
while ~end_sim
    % Append History
    x_history= cat(2,x_history,x0);
    % Compute control action
    u = clf_cbf(x0);
    % Integrate forward
    t_end_t = min(t + dt, t0+T);
%     [ts_t, xs_t] = ode45(@(t, x) dynamics(t, x, u), ...
%             [t, t_end_t], x0);
%     t = ts_t(end);
%     x0 = xs_t(end, :)';
    if t_end_t == t0+T
        end_sim = 1;
    end
    t = t_end_t;
    x0 = integrate_forward(t, x0, u);
    % Append history
end


% Plot Actions
x_obst = [32.0, 25.0;
          -10, -10];
r = [7, 7];
figure
plot(x_history(1, :), x_history(2, :)); hold on;
for i = 1:size(x_obst, 1)
    draw_circle(x_obst(i, :), r(i));
end
lim_min = min(min(x_history(1, :)), min(x_history(2, :)));
lim_max = max(max(x_history(1, :)), max(x_history(2, :)));
lim_min = min([lim_min, x_obst(:, 1)'-r, x_obst(:, 2)'-r]);
lim_max = max([lim_max, x_obst(:, 1)'+r, x_obst(:, 2)'+r]);
xlim([lim_min, lim_max]);
ylim([lim_min, lim_max]);
xlabel('x [m]')
ylabel('y [m]')
axis equal

function h = draw_circle(center,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + center(1);
    yunit = r * sin(th) + center(2);
    h = plot(xunit, yunit);
    hold off
end


%% Define Phyisical Constraints
function xdot = dynamics(t, x,u)
    % compute derivatives
    theta = x(4);
    xdot = [x(3) * cos(theta);
            x(3) * sin(theta);
            u(1);
            u(2);
            ];
end
function x = integrate_forward(t, x_0, u)
    substeps = 10;
    dt = 0.1;
    % Solve the MPC problem to get the next state
    % Integrate forward
    x = x_0;
    xdot = dynamics(t, x,u);
    for i=1:substeps
        x = x +  xdot*dt/substeps;
    end
end
function [u] = clf_cbf(x)
    opti = casadi.Opti(); % Optimization problem
    %% Problem Parameters
    x_goal = [45.0, 25.0, 2.0, 0.0];
    x_obst = [32.0, 25.0
              -10, -10];  
    r = [7, 7];
    %% Dynamics Parameters
    n_states = 4;
    n_controls = 2;
    
    control_bounds = [1, 0.5]';
    n_clf = 2;  % Number of CLF constraints [V_speed, V_angle]
    n_cbf = 4;  % Number of CBF constraints [b_radius, b_v_min, b_v_max]
    n_clf_slack = n_clf;  % Number of CLF slack variables
    n_cbf_slack = 2;  % Number of CBF slack variables
    slack_weights = [1., 1.,  10000 * ones(size(r))];
    rel_degree = [2,2, 1, 1];  % Relative degree of the CBFs [distance, v_min, v_max]

    
    %% Setup optimization variables
    u = opti.variable(n_controls,1); % control variables
    delta = opti.variable(n_clf_slack,1); % slack variables
    slack = opti.variable(n_cbf_slack,1);
    %% Decompose States
    px = x(1); py = x(2); v = x(3); theta = x(4);
    %% Compute cbf-clf constraints
    % Tunning rates
    p1 = 1;
    p2 = 50;
    clf_rate = 1;
    % Make sure that the list of relative degrees is correct
    if length(rel_degree) ~= n_cbf
        disp("Error: the number of relative degrees must be equal to the number of CBFs")
    end
    
    % -------- Extract Parameters from NN --------
    parameters = [1.,1.,1., 1., 1.,50.,1., 50., 1., 1.];
    % Compute the total number of cbf rates corresponding to the relative degree
    n_cbf_rates = sum(rel_degree);
    % Extract control weights from NN
    cntrl_weights = parameters(1:n_controls);
    % Extract the CLF rates from the NN
    clf_rates = parameters(n_controls+1:n_controls + n_clf);
    % Extract the CBF rates from the NN
    cbf_rates = parameters(n_controls + n_clf+1:end);
    % -------- Compute CLF and CBF Constraint Parameters --------
    % Compute CLF parameters
    % TODO this can be called for automatic differentiation given a vector of Lyapunov functions
    LfV = zeros(n_clf, 1);
    LgV = [2 * (v - x_goal(3));
           -2*(cos(theta)*(px - x_goal(1)) + sin(theta)*...
            (py - x_goal(2)))*(cos(theta)*(py - x_goal(2)) - ...
            sin(theta)*(px - x_goal(1)))];
    V_speed = (v-x_goal(3))^2;
    V_angle = (cos(theta)*(py-x_goal(2))-sin(theta)*(px-x_goal(1)))^2;
    
    V = [V_speed;V_angle];

    A_clf = diag(LgV);
    b_clf = -LfV - clf_rates' .* V;
    % Add clf constraints
    opti.subject_to( A_clf * u <= b_clf+delta)
    % Compute CBF parameters
    A_cbf = zeros(n_cbf, n_controls);
    b_cbf = zeros(n_cbf, 1);
    % Distance from Obstacle

    for i=1:size(x_obst,1)
        b_dist = (px - x_obst(i,1))^2 + (py - x_obst(i,2)) ^ 2 - r(i) ^ 2;
        LgLfb_dist = [2 * cos(theta) * (px - x_obst(i,1)) + 2 * sin(theta) * (py - x_obst(i,2)), ...
                       2 * v * cos(theta) * (py - x_obst(i,2)) - 2 * v * sin(theta) * (px - x_obst(i,1))];
        Lfb_dist = 2 * (px - x_obst(i,1)) * v * cos(theta) + ...
                   2 * (py - x_obst(i,2)) * v * sin(theta);
        Lf2b_dist = 2 * v^2;
        A_cbf(i, 1:n_controls) = -LgLfb_dist;
        b_cbf(i) = Lf2b_dist + (p1 + p2) * Lfb_dist + p1 * p2 * b_dist;
    end
    % Velocity Bounds Barrier
    b_v = [v - 0.1; 2 - v];
    Lgb_v = [1;-1];
    cbf_rates_v = [p1,p1]';
    n_dist = length(x_obst);
    A_cbf(n_dist+1:end, 1) = -Lgb_v;
    b_cbf(n_dist+1:end) = cbf_rates_v .* b_v;

    %% Create constraints
    % Add Actuation Limits
    opti.subject_to(-control_bounds<=u<=control_bounds)
    % Add clf constraints
    slack_cbf = [slack;zeros(n_cbf-n_cbf_slack,1)];
    opti.subject_to( A_cbf * u <= b_cbf+slack_cbf)
    % speed constraints 
    
    % Compute objective
    % Objective matrix
    U = [u;delta;slack];
    
    weights = diag([ones(1,n_controls), slack_weights]);
    objective = 0.5*U'*weights* U;

    opti.minimize(objective)

    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
%       opts = struct();
%       opts.qpsol = "qpoases";
% 
%       opti.solver('qpoases')

    try
        sol = opti.solve();   % actual solve
        u = sol.value(u);
    catch
        u= [0,0]';
    end


end
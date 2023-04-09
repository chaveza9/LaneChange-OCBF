clear all
close all
clc
if ismac
    addpath('casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('casadi-windows-matlabR2016a-v3.5.5')
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
%     # Solve the MPC problem to get the next state
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
    n_clf = 2;
    n_cbf = 2;
    n_cbf_clf = n_cbf + n_clf;
    control_bounds = [1, 0.5]';
    
    %% Setup optimization variables
    u = opti.variable(n_controls,1); % control variables
    delta = opti.variable(n_controls,1); % slack variables
    slack = opti.variable(size(x_obst,1));
    %% Decompose States
    px = x(1); py = x(2); v = x(3); theta = x(4);

    x_s = casadi.MX.sym('x',n_states);
    x_o = casadi.MX.sym('x_o',2);
    r_o = casadi.MX.sym('r_o',1);
    cbf = casadi.Function('cbf',{x_s,x_o,r_o},...
              {(x_s(1) - x_o(1))^2 + (x_s(2) - x_o(2))^2 - r_o^2},...
              {'x','x_obst','r_obst'},{'b'});

    d_cbf = cbf.jacobian_old(0,0);
    dd_cbf = cbf.hessian_old(0,0);
    
    %% Compute cbf-clf constraints
    % Tunning rates
    p1 = 1; p2 = 50;
    clf_rate = 1;
    weight_slack_cbfs = 10000 * ones(size(r));
    weight_slack_clf = [1,1];
    weight_slack = [weight_slack_clf, weight_slack_cbfs];
    % Compute conditions CLF
    V_speed = (v-x_goal(3))^2;
    V_angle = (cos(theta)*(py-x_goal(2))-sin(theta)*(px-x_goal(1)))^2;
    
    V = [V_speed;V_angle];
    LgV = [2 * (v - x_goal(3));
           -2*(cos(theta)*(px - x_goal(1)) + sin(theta)*...
            (py - x_goal(2)))*(cos(theta)*(py - x_goal(2)) - ...
            sin(theta)*(px - x_goal(1)))];
    LfV = 0;
    opti.subject_to(LgV.*u + clf_rate*V <= delta)
    % Compute conditions CBF
    for i = 1:size(x_obst,1)
        Lgb = [cos(theta)*(2*px - 2*x_obst(i,1)) + sin(theta)*(2*py - 2*x_obst(i,2)), ...
             v*cos(theta)*(2*py - 2*x_obst(i,2)) - v*sin(theta)*(2*px - 2*x_obst(i,1))];
        distance = (px - x_obst(i,1))^2 + ...
            (py - x_obst(i,2))^2 - r(i)^2;
        Lfb = 2*(px-x_obst(i,1))*v*cos(theta) + ...
            2*(py-x_obst(i,2))*v*sin(theta);
        Lf2b = 2*v^2; %+ p1*Lfb;
        cbf = Lfb + p1*distance;
        if Lgb
          opti.subject_to(Lf2b+Lgb*u+(p2+p1)*Lfb+p1*p2*distance>=-slack(i)) % CBF
        end
    end
% Add Speed Limits
    opti.subject_to(-u(1)+p1*(2 - v)>=0)
    opti.subject_to(u(1)+p1*(v-0.1)>=0)
    % Add Actuation Limits
    opti.subject_to(-control_bounds<=u<=control_bounds)

    % Add Slack Variables
   
    U = [u;delta;slack];
    % Define Objective
    H = [eye(n_controls), zeros(n_controls,length(U)-n_controls);
        zeros(length(U)-n_controls,n_controls), diag(weight_slack)];
    
    objective = 0.5*U'*H*U;

    opti.minimize(objective)

    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend

    sol = opti.solve();   % actual solve
    u = sol.value(u);


end
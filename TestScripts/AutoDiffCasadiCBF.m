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
T  = 50;
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
function x_dot = dynamics(t, x,u)
    % compute derivatives
    x_dot = f(x)+g(x)*u;
end

function x_dot = f(x)
    x_dot = [x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
end

function val = g(x)
    val = [zeros(2,2);
         eye(2)];
end


function [A, B] = compute_lie_derivative_2nd_order(x, barrier_fun, alpha_fun_1, alpha_fun_2)

    %% Define CBF functions
    n_states = length(x);
    x_s = casadi.MX.sym('x',n_states);
    %% Compute cbf constraints
    alpha_0 = barrier_fun(x_s);
    b = casadi.Function('alpha0',{x_s},...
              {alpha_0});
    % Compute constraint parameters 
    db_dx = b.jacobian_old(0,0);
    Lfb = casadi.Function('Lfb',{x_s},...
              {db_dx(x_s)*f(x_s)});
    db2_dx = Lfb.jacobian_old(0,0);
    Lf2b = casadi.Function('Lf2b',{x_s},...
              {db2_dx(x_s)*f(x_s)});
    LgLfb = casadi.Function('LgLfb',{x_s},...
              {db2_dx(x_s)*g(x_s)});
    % Compute HO terms
    psi0 = b(x_s);
    psi1 = casadi.Function('psi_1',{x_s},...
              {Lfb(x_s)+alpha_fun_1(psi0)});
    psi1_dot = psi1.jacobian_old(0,0);
    
    psi2 = casadi.Function('psi_1',{x_s},...
              {psi1_dot(x_s)*f(x_s)+alpha_fun_2(psi1(x_s))});

    % construct constraints
    A = (LgLfb(x));
    B = (Lf2b(x) + psi2(x));   
end
function [A, B] = compute_lie_derivative_1st_order(x, barrier_fun, alpha_fun_1)

    %% Define CBF functions
    n_states = length(x);
    x_s = casadi.MX.sym('x',n_states);
    %% Compute cbf constraints
    alpha_0 = barrier_fun(x_s);
    b = casadi.Function('alpha0',{x_s},...
              {alpha_0});
    % Compute constraint parameters 
    db_dx = b.jacobian_old(0,0);
    Lfb = casadi.Function('Lfb',{x_s},...
              {db_dx(x_s)*f(x_s)});
    Lgb = casadi.Function('LgLfb',{x_s},...
              {db_dx(x_s)*g(x_s)});
    % Compute HO terms
    psi0 = b(x_s);
    psi1 = casadi.Function('psi_1',{x_s},...
              {Lfb(x_s)+alpha_fun_1(psi0)});

    % construct constraints
    A = (Lgb(x));
    B = (Lfb(x) + psi1(x));   
end

function x = integrate_forward(t, x_0, u)
    substeps = 1;
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
    n_controls = 2;
    n_clf = 2;
    n_cbf = size(x_obst,1)+2;
    n_cbf_clf = n_cbf + n_clf;
    control_bounds = [1, 0.5]';
    
    %% Setup optimization variables
    u = opti.variable(n_controls,1); % control variables
    slack_clf = opti.variable(n_controls,1); % slack variables
    slack_cbf = opti.variable(size(x_obst,1));
    
    %% Compute cbf-clf constraints
    % Tunning rates
    p1 = 1; p2 = 50;
    clf_rate = 1;
    weight_slack_cbfs = 10000 * ones(size(r));
    weight_slack_clf = [1,1];
    weight_slack = [weight_slack_clf, weight_slack_cbfs];
   
    %% Compute conditions CLF
    % define buffer for constraints
    A_clf = zeros(n_clf, n_controls);
    b_clf = zeros(n_clf, 1);
    % define Lyapunov functions
    V_speed = @(x) (x(3)-x_goal(3))^2;
    V_angle = @(x) (cos(x(4))*(x(2)-x_goal(2))-sin(x(4))*(x(1)-x_goal(1)))^2;
    V = {V_speed, V_angle};
    for i =1:length(V)
        % Define lyapunov function
        v_i = V{i};
        % Define alpha function
        alpha_1 = @(psi) clf_rate*psi;
        % Compute clf constraints
        [G, F] = compute_lie_derivative_1st_order(x, v_i, alpha_1);
        % Populate Constraint Matrix
        A_clf(i,:) = full(G);
        b_clf(i) = -full(F);
    end
    % Form CBF constraint matrix
    opti.subject_to(A_clf * u <= b_clf+slack_clf)
    %% Compute conditions CBF
    % Define buffer for constraints
    A_cbf = zeros(n_cbf, n_controls);
    b_cbf = zeros(n_cbf, 1);
    % define barrierfunctions
    v_min = @(x) x(3)-0.1;
    v_max = @(x) 2 - x(3);
    B = {};
    % Concatenate barrier functions in accordance to number of obstacles
    for i =1:size(x_obst,1)
        dist_i = @(x) (x(1) - x_obst(i,1))^2 + (x(2) - x_obst(i,2))^2 - r(i)^2;
        B{i} = dist_i;
    end
    B = cat(2,B,{v_min, v_max});
    for i =1:length(B)
        % Define barrier function
        b_i = B{i};
        if i ==1 || i ==2 % Distance CBF
            % Define alpha functions
            alpha_1 = @(psi) p1*psi;
            alpha_2 = @(psi) p2*psi;
            % Compute CBF constraints
            [G, F] = compute_lie_derivative_2nd_order(...
                x, b_i, alpha_1, alpha_2);
        else
            % Define alpha functions
            alpha_1 = @(psi) p1*psi;
            % Compute CBF constraints
            [G, F] = compute_lie_derivative_1st_order(...
                x, b_i, alpha_1);
        end
        % Populate cbf constraint matrix
        A_cbf(i, :) = -full(G);
        b_cbf(i) = full(F);
    end
    % Form CBF constraint matrix
    opti.subject_to(A_cbf * u <= b_cbf+[slack_cbf;zeros(2,1)])
    % Add Actuation Limits
    opti.subject_to(-control_bounds<=u<=control_bounds)

    % Add Slack Variables
   
    U = [u;slack_clf;slack_cbf];
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
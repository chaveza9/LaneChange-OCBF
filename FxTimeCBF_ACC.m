clear all
close all
clc
if ismac
    addpath('./casadi-3.6.0-osx64-matlab2018b')
else
    addpath('.\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*


% Initial Conditions
d_0 = 100; %[m]
vl_0 = 17; %[m/s]
vf_0 = 20;  %[m/s]
x0 = [vf_0 , vl_0, d_0]';
% Buffer history
x_history = [];
u_history = [];
delta_history=[];
t = 0;
dt = 0.1;
t0 = 0;
T  = 100;
tfx = 10;
end_sim = 0;
t_end_t = 0;
while ~end_sim
    % Append History
    x_history= cat(2,x_history,x0);
    % Compute control action
    time = max(tfx-t_end_t,0.01);
    % time = tfx;
    [u, delta_1, delta_2 ]= clf_cbf(x0, time);
    u_history= cat(2,u_history,u);
    delta_history= cat(2,delta_history,[delta_1;delta_2]);
    % Integrate forward
    t_end_t = min(t + dt, t0+T);
    if t_end_t == t0+T
        end_sim = 1;
    end
    t = t_end_t;
    x0 = integrate_forward(t, x0, u);
end


% Plot Actions
figure
t_hist = linspace(0, t_end_t, length(x_history));
plot(t_hist,x_history(1, :)); hold on;
plot(t_hist,x_history(2, :)); 
grid minor
legend('v_f', 'v_l')
xlabel('Time [s]')
ylabel('Speed [m/s]')
xlim([0, t_end_t])
ylim([0,30])


figure
t_hist = linspace(0, t_end_t, length(x_history));
plot(t_hist,x_history(3, :)); hold on;
hs = 1.8*x_history(2, :)-x_history(3, :);
plot(t_hist,hs); 
grid minor
legend('D(x)', 'H_s')
xlabel('Time [s]')
ylabel('Distance [m]')
xlim([0, t_end_t])
% ylim([0,30])
figure
t_hist = linspace(0, t_end_t, length(u_history));
plot(t_hist,delta_history(1,:)); hold on
plot(t_hist,delta_history(2,:)); 
grid minor
legend('\delta_1', '\delta_2')
xlabel('Time [s]')
xlim([0, t_end_t])
figure
t_hist = linspace(0, t_end_t, length(u_history));
plot(t_hist,u_history); hold on
grid minor
legend('accel [m/s^2]')
xlabel('Time [s]')
xlim([0, t_end_t])
% axis equal


%% Define Phyisical Constraints
function x_dot = dynamics(t, x,u)
    % compute derivatives
    x_dot = f(x)+g(x)*u;
end

function x_dot = f(x)
    % Mass
    M = 1650; %[kg] mass following vehicle
    % Leader Acceleration
    al = 0.0;
    aL = al*9.81;
    % x = [vf , vl, d] [following veh velocity, leader veh velocity,
    % distance between vehicles]
    vf = x(1); vl = x(2); d = x(3);
    % Compute Drag
    Fr = drag(x);
    x_dot = [-Fr/M, aL, vl-vf]';
end

function val = g(x)
    M = 1650; %[kg] mass following vehicle
    val = [ 1/M, 0, 0]';
end
function Fr = drag(x)
    % Drag coeff
    f0 = 0.1; %[N]
    f1 = 5; % [Ns/m]
    f2 = 0.25; % [Ns^2/m^2]
    % Mass
    % x = [vf , vl, d] [following veh velocity, leader veh velocity,
    % distance between vehicles]
    vf = x(1);
    % Compute Drag
    Fr = f0 + f1*vf + f2*vf^2;
end

function [Lgb, Lfb] = compute_lie_derivative_1st_order(x, barrier_fun)

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
    % psi0 = b(x_s);
    % psi1 = casadi.Function('psi_1',{x_s},...
    %           {Lfb(x_s)+alpha_fun_1(psi0)});

    % construct constraints
    % A = (Lgb(x));
    % B = (Lfb(x) + psi1(x));   
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
function [u, slack_clf, slack_cbf] = clf_cbf(x, time)
    opti = casadi.Opti(); % Optimization problem
    %% Problem Parameters
    vd = 22; % [m/s]
    %% Dynamics Parameters
    n_controls = 1;
    n_clf = 1;
    n_cbf = 1;
    n_vars = n_controls + n_clf + n_cbf; 
    M = 1650;
    control_bounds = 0.4*M*9.81;
    
    %% Setup optimization variables
    u = opti.variable(n_controls,1); % control variables
    slack_clf = opti.variable(n_controls,1); % slack variables
    slack_cbf = opti.variable(n_cbf, 1);
    Z = [u, slack_clf, slack_cbf]';
    %% Compute cbf-clf constraints
    weight_slack_cbfs = 1 ;
    weight_slack_clf = 1;
    weight_slack = [weight_slack_clf, weight_slack_cbfs];
    weight_slack_linear = [0, weight_slack_clf, 0];
    % CLF params
    T_fx = time; %[sec]
    mu = 5;
    gamma_1 = 1 + 1/mu;
    gamma_2 = 1 - 1/mu;
    % CBF
    Tau = 1.8; %[s]

   
    %% Compute conditions CLF
    % define buffer for constraints
    A_clf = zeros(n_clf, n_vars);
    b_clf = zeros(n_clf, 1);
    % define Lyapunov functions
    V_speed = @(x) (x(1)-vd)^2;
    V = {V_speed};
    for i =1:length(V)
        % Define lyapunov function
        h_g = V{i};
        % Define alpha function
        T_ud = T_fx;
        alpha = mu*pi/(2*T_ud);
        % Compute clf constraints
        [Lgh_g, Lfh_g] = compute_lie_derivative_1st_order(x, h_g);
        % L = [Lgh_g(x) -h_g(x) 0];
        L = [Lgh_g(x) -1 0];
        R = Lfh_g(x)+ alpha*max(0,h_g(x))^gamma_1 + ...
            alpha*max(0,h_g(x))^gamma_2;
        % Populate Constraint Matrix
        A_clf(i,:) = full(L);
        b_clf(i) = -full(R);
    end
    % Form CBF constraint matrix
    opti.subject_to(A_clf * Z <= b_clf)
    %% Compute conditions CBF
    % Define buffer for constraints
    A_cbf = zeros(n_cbf, n_vars);
    b_cbf = zeros(n_cbf, 1);
    % define barrierfunctions
    b_safety =@(x) Tau*x(1)-x(3);
    B = {b_safety};
    for i =1:length(B)
        % Define barrier function
        h_s = B{i};
        % Compute CBF constraints
        [Lgh_s, Lfh_s] = compute_lie_derivative_1st_order(x, h_s);
        % Evaluate Constraints
        G = [Lgh_s(x) 0 h_s(x)];
        R = Lfh_s(x);
        % Populate cbf constraint matrix
        A_cbf(i, :) = full(G);
        b_cbf(i) = -full(R);
        opti.subject_to(Lfh_s(x)+Lgh_s(x)*u <= -slack_cbf*h_s(x))
    end
    opti.subject_to(slack_cbf>=0)
    opti.subject_to(slack_cbf<=1/0.1)
    % Form CBF constraint matrix
    % opti.subject_to(A_cbf * Z <= b_cbf)
    % Add Actuation Limits
    opti.subject_to(-control_bounds<=u<=control_bounds)

    % Define Objective
    H = [eye(n_controls), zeros(n_controls,length(Z)-n_controls);
        zeros(length(Z)-n_controls,n_controls), diag(weight_slack)];
    F =  0*weight_slack_linear;
    objective = 0.5*Z'*H*Z+F*Z;

    H = [2/M^2 0 0;
         0     2 0
         0     0 200];
    F = 2*[drag(x)/M^2 1 0];
    objective = 0.5*Z'*H*Z+F*Z;

    opti.minimize(objective)

    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend

    sol = opti.solve();   % actual solve
    u = sol.value(u);
    slack_cbf = sol.value(slack_cbf);
    slack_clf = sol.value(slack_clf);

end
function [tf_var,x_C_f,v_C_f,posStates,speedStates,accTraj,cost] = solve_v_des(x_C_0,v_C_0,x_U_0,v_U_0,N,tf, xf)
% Constraints
u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
u_min = -7;   % Vehicle i min acceleration [m/s^2]
v_min = 15;   % Vehicle i min velocity [m/s]
v_max = 35;   % Vehicle i max velocity [m/s]
%parameters


DesSpeed = 30;
alpha_energy = 0.2;
gamma_energy = (alpha_energy)/max([u_max, u_min].^2);
gamma_speed = (1-alpha_energy)/max([v_max-DesSpeed, v_min-DesSpeed].^2);
ReactionTime = 0.9;
SafetyDistance = 1.5;

%%  Numerical Solution
opti = casadi.Opti(); % Optimization problem

% Define decision variables
x_var = opti.variable(2,N+1); % state trajectory
pos   = x_var(1,:);
speed = x_var(2,:);
u_var = opti.variable(1,N);   % control trajectory (throttle)
% Front Obstacle Vehicle
% X_obs_var = opti.parameter(2,1); % Obstacle Vehicle Initial Conditions
X_obs_var = [x_U_0,v_U_0];
x_u = X_obs_var(1);
v_u = X_obs_var(2);
% Terminal Time Parametric Variable
% tf_var = opti.parameter(); % Vehicle terminal time
% v_des_var = opti.parameter();
tf_var = tf;
v_des_var = DesSpeed;
% Define dynamic constraints
c = @(u, x) u^2; % Cost Acceleration
f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
% Define objective funciton integrator
% Integrate using RK4
dt = tf_var/N;
cost = 0;

for k=1:N % loop over control intervals
    % Forward integration
    x_next = runge_kutta4(f, x_var(:,k), u_var(:,k), dt);
    cost = cost + 0.5*gamma_energy*runge_kutta4(c, u_var(:,k), x_var(:,k), dt);
    % cost = cost + 0.5*gamma_speed*runge_kutta4(c,(speed(:,k)-v_des_var),x_var(:,k),dt);
    % Impose multi-shoot constraint
    opti.subject_to(x_var(:,k+1)==x_next); % close the gaps
    % Impose Front Vehicle Safety Constraint
    x_obs_k = x_u+v_u*dt*k;
    % safety constraint
    % opti.subject_to(x_obs_k - pos(k+1)>=speed(k+1)*ReactionTime+SafetyDistance);
end

% ----- Objective function ---------
opti.minimize(cost);
% --------- Define path constraints ---------
opti.subject_to(v_min<=speed<=v_max);     %#ok<CHAIN> % track speed limit
opti.subject_to(u_min<=u_var<=u_max);     %#ok<CHAIN> % control is limited
% --------- Define Initial Condition -------
curr_state = [x_C_0,v_C_0]';
opti.subject_to(x_var(:,1) == curr_state)
% --------- Define Terminal Condition ------
opti.subject_to(pos(end) == xf)
opti.subject_to(speed(end) == DesSpeed)
% Populate Parameters
x_obs_0 = [x_U_0,v_U_0];
% opti.set_value(X_obs_var, x_obs_0)
% opti.set_value(tf_var, tf)
% opti.set_value(v_des_var, DesSpeed)
% Warm Start solver
opti.set_initial(u_var, 3.3);
%% Solver Parameter Options

opti.solver('ipopt',struct('print_time',1,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',3,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
% opti.solver('bonmin',struct('print_time',1,'bonmin',...
%     struct('max_iter',100000))); % set numerical backend
sol = opti.solve();   % actual solve

x_C_f = sol.value(pos(end));
v_C_f = sol.value(speed(end));
posStates = sol.value(pos);
speedStates = sol.value(speed);
accTraj = sol.value(u_var);
cost = sol.value(cost);


function x_next = runge_kutta4(f, x, u, dt)
% Runge-Kutta 4 integration
k1 = f(x,         u);
k2 = f(x+dt/2*k1, u);
k3 = f(x+dt/2*k2, u);
k4 = f(x+dt*k3,   u);
x_next = x + dt/6*(k1+2*k2+2*k3+k4);
end

end


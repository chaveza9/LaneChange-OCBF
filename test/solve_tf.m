function [tf,x_C_f,v_C_f,posStates,speedStates,accTraj,cost] = solve_tf(x_C_0,v_C_0,x_U_0,v_U_0,N,t0)
% Constraints
u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
u_min = -7;   % Vehicle i min acceleration [m/s^2]
v_min = 15;   % Vehicle i min velocity [m/s]
v_max = 35;   % Vehicle i max velocity [m/s]
%parameters


DesSpeed = 30;
alpha_time = 0.55;
alpha_energy = 0.2;
beta_time = alpha_time;
beta_speed = (1-alpha_time-alpha_energy);%/max([v_max-DesSpeed, v_min-DesSpeed].^2);
beta_energy = (alpha_energy)/max([u_max, u_min].^2);
ReactionTime = 0.6;
SafetyDistance = 1.5;

%%  Numerical Solution
opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
tf = opti.variable(); % state trajectory
X = opti.variable(2,N+1); % state trajectory
pos   = X(1,:);
speed = X(2,:);
U = opti.variable(1,N);   % control trajectory (throttle)


% Define dynamic constraints
c = @(u, x) u^2; % Cost Acceleration
f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
l = @(u,x) u;
% Define objective funciton integrator
% Integrate using RK4
dt = tf/N;
cost = beta_time*tf;
cost_v = beta_speed*(speed(end)-DesSpeed)^2;


for k=1:N % loop over control intervals
    % Forward integration
    x_next = runge_kutta4(f, X(:,k), U(:,k), dt);
    cost = cost + 0.5*beta_energy*runge_kutta4(c, U(:,k), X(:,k), dt);
    %cost_v = cost_v + beta_speed*runge_kutta4(l,(speed(:,k)-DesSpeed)^2,X(:,k),dt);
    % Impose multi-shoot constraint
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
    % Impose Front Vehicle Safety Constraint
    x_U = x_U_0+v_U_0*dt*k;
    % safety constraint
    opti.subject_to(x_U - pos(k+1)>=speed(k+1)*ReactionTime+SafetyDistance);
end

cost = cost+cost_v;
% ----- Objective function ---------
opti.minimize(cost);
% --------- Define path constraints ---------
opti.subject_to(v_min<=speed<=v_max);     %#ok<CHAIN> % track speed limit
opti.subject_to(u_min<=U<=u_max);     %#ok<CHAIN> % control is limited
% --------- Define Boundary conditions ---------
opti.subject_to(pos(1)==x_C_0);   % start at position 0 ...
opti.subject_to(speed(1)==v_C_0); % initial speed
opti.subject_to(tf>0); %time must be positive
%opti.subject_to(pos(end)==x_2_0+v_2_0*tf);
% Warm Start solver
% opti.set_initial(speed, 25);
opti.set_initial(U, 3.3);
opti.set_initial(tf, 5);
%% Solver Parameter Options
% opti.solver('ipopt',struct('print_time',obj.Verbose,'ipopt',...
%     struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',0,...
%     'acceptable_obj_change_tol',1e-6))); % set numerical backend
opti.solver('ipopt',struct('print_time',1,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',3,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
% opti.solver('bonmin',struct('print_time',1,'bonmin',...
%     struct('max_iter',100000))); % set numerical backend
sol = opti.solve();   % actual solve
tf = sol.value(tf)+t0;
x_C_f = sol.value(pos(end));
v_C_f = sol.value(speed(end));
posStates = sol.value(pos);
speedStates = sol.value(speed);
accTraj = sol.value(U);
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


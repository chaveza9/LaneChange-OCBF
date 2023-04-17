% Problem 15 Sequential Maneuver numerical solution using Casadi
% -----------------------------------------

clear all
close all
clc
if ismac
    addpath('../casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('G:\My Drive\PhD\Research\CODES\Honda\Multiple Lane Change Maneuvers\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*

%% Define Phyisical Constraints
u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
u_min = -7;   % Vehicle i min acceleration [m/s^2]
v_min = 15;   % Vehicle i min velocity [m/s]
v_max = 35;   % Vehicle i max velocity [m/s]
%% Define Initial States
% Vehicle C initial States;
v_C_f = 29.9326;
x_C_f = 235.66;
% Vehicle 2 States;
v_2_0 = 28.619;
x_2_0 = 160.2765;
%% Tunning Variables
phi = 0.6; % [seconds] Intervehicle Reaction Time
delta = 1.5; % [meters] intervehicle distance
N = 100; % number of control intervals
v_des = 33; % free flow desired speed;
tol_speed = 2; % speed tolerance
%%  Numerical Solution
opti = casadi.Opti(); % Optimization problem
T = 2.611; % final time
X = opti.variable(2,N+1); % state trajectory
pos   = X(1,:);
speed = X(2,:);
Accel = opti.variable(1,N);   % control trajectory (throttle)

% ---- objective          ---------
L = 0;
% ---- dynamic constraints --------
c = @(u) 0.5*u.^2;
f = @(x,u) [x(2);u;c(u)]; % dx/dt = f(x,u)
alpha = 0.5;
beta = (1-alpha) * (max([u_min, u_max].^ 2)) / alpha; 
beta = 1;
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         Accel(:,k));
   k2 = f(X(:,k)+dt/2*k1(1:end-1), Accel(:,k));
   k3 = f(X(:,k)+dt/2*k2(1:end-1), Accel(:,k));
   k4 = f(X(:,k)+dt*k3(1:end-1),   Accel(:,k));
   x_next = X(:,k) + dt/6*(k1(1:end-1)+2*k2(1:end-1)+2*k3(1:end-1)+k4(1:end-1)); 
   L = L + dt/6*(k1(end)+2*k2(end)+2*k3(end)+k4(end));

   L = L*1;
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end
L = L+beta*(speed(end)-v_des)^2;

% Compute Vehicle U position
opti.subject_to(x_C_f-pos(1,end)>=(v_2_0*phi+delta)); % close the gaps
opti.minimize(L);
% ---- path constraints -----------
opti.subject_to(v_min<=speed<=v_max);           % track speed limit
opti.subject_to(u_min<=Accel<=u_max);           % control is limited

% ---- boundary conditions --------
opti.subject_to(pos(1)==x_2_0);   % start at position 0 ...
opti.subject_to(speed(1)==v_2_0); % ... from stand-still 


% ---- initial values for solver ---
opti.set_initial(speed, 25);
opti.set_initial(Accel, 3.3);

% ---- solve NLP              ------
opti.solver('ipopt',struct('print_time',1,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',3,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
sol = opti.solve();   % actual solve

% ---- post-processing        ------
Time = T;
figure
subplot(3,1,1)
time = linspace(0,Time,N+1);
plot(linspace(0,Time,N+1),sol.value(pos))
hold on 
grid minor
xu = x_C_f*ones(size(time));
v_end= sol.value(speed);
xu_safe=v_end(end)*phi+delta;
plot(time,xu,".-")
plot(time,xu_safe,"--")
ylabel("Position x(t) [m]")
legend('CAV 2', 'CAV C', 'Safety Distance','Location','best')
subplot(3,1,2)
plot(linspace(0,Time,N+1),sol.value(speed))
hold on 
grid minor
ylabel("Speed v(t) [m/s]")
subplot(3,1,3)
plot(linspace(0,Time,N),sol.value(Accel))
hold on 
grid minor
ylabel("Accel u(t) [m/s^2]")
xlabel("Time [sec]")


opti.delete
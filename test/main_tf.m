clc
clear
close all

if ismac
    addpath('/Users/annili/Desktop/PhD/casadi-osx-matlabR2015a-v3.5.5/')
else
    addpath('G:\My Drive\Mixed Traffic\casadi-windows-matlabR2016a-v3.5.5')
end

import casadi.*

%% Settings
u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
u_min = -7;   % Vehicle i min acceleration [m/s^2]
v_min = 5;   % Vehicle i min velocity [m/s]
v_max = 35;   % Vehicle i max velocity [m/s]

% Vehicle U initial States;
v_U_0 = 17;
x_U_0 = 100;
% Vehicle C States;
v_C_0 = 24;
x_C_0 = 50;
% Vehicle F States;
v_F_0 = 28;
x_F_0 = 115;
% Vehicle 1 States;
v_1_0 = 28;
x_1_0 = 85;
% Vehicle 2 States;
v_2_0 = 24;
x_2_0 = 60;
% Vehicle 3 States;
v_3_0 = 24;
x_3_0 = 25;
% Vehicle B States;
v_B_0 = 24;
x_B_0 = 0;

% Parameters
N = 100;
t0 = 0;
T = 20;
gamma_x = 0.5;
gamma_v = 0.5;
phi = 0.4;
delta = 10;
M = 1000;



%% solve optimal terminal time tf^* and terminal position 
[tf,x_C_f,v_C_f,posCStates,speedCStates,accTrajC,costC] = solve_tf(x_C_0,v_C_0,x_U_0,v_U_0,N,t0);

t = 0:(tf-t0)/N:tf-t0;
posFStates = x_F_0 + v_F_0*t;
posBStates = x_B_0 + v_B_0*t;
x_1_f_ideal = x_1_0 + v_1_0*tf;
x_2_f_ideal = x_2_0 + v_2_0*tf;
x_3_f_ideal = x_3_0 + v_3_0*tf;



%% Solve NLMIP
opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
x_1 = opti.variable(); % terminal state
v_1 = opti.variable();
% pos1   = X1(1,:);
% speed1 = X1(2,:);

x_2 = opti.variable(); % state trajectory
v_2 = opti.variable();
% pos2   = X2(1,:);
% speed2 = X2(2,:);

x_3 = opti.variable(); % state trajectory
v_3 = opti.variable();
% pos3   = X3(1,:);
% speed3 = X3(2,:);
 
%x = MX.sym('x', 'int'); 
B1 = opti.variable();    % binary variable
B2 = opti.variable();
B3 = opti.variable();
%discrete = [false;false;false;false;false;false;true;true;true];
discrete = [0;0;0;0;0;0;1;1;1];
% Define dynamic constraints
f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
% Integrate using RK4
dt = tf/N;
v_d = 30;
gamma = 0.5;
% Disruption
% [gamma_x, gamma_v] = compute_disruption_norm_cons...
%     (gamma, v_max, v_min, v_1_0, v_d, tf);
disruption1 = gamma_x*(x_1-x_1_f_ideal)^2+gamma_v*(v_1-v_d)^2;
% [gamma_x, gamma_v] = compute_disruption_norm_cons...
%     (gamma, v_max, v_min, v_2_0, v_d, tf);
disruption2 = gamma_x*(x_2-x_2_f_ideal)^2+gamma_v*(v_2-v_d)^2;
% [gamma_x, gamma_v] = compute_disruption_norm_cons...
%     (gamma, v_max, v_min, v_3_0, v_d, tf);
disruption3 = gamma_x*(x_3-x_3_f_ideal)^2+gamma_v*(v_3-v_d)^2;

cost = disruption1 + disruption2 + disruption3;
% ----- Objective function ---------
opti.minimize(cost);

% --------- Define Boundary conditions ---------
opti.subject_to(posFStates(end) - x_1 >= phi*v_1 + delta);
opti.subject_to(x_3 - posBStates(end) >= phi*v_B_0 + delta);
%opti.subject_to(posFStates(end) - x_C_f >= phi*v_C_f + delta);
%opti.subject_to(x_C_f - posBStates(end) >= phi*v_B_0 + delta);
opti.subject_to(x_1 - x_2 >= phi*v_2 + delta);
opti.subject_to(x_2 - x_3 >= phi*v_3 + delta);
opti.subject_to(x_C_f - x_1 + (1-B1)*M >= phi*v_1 + delta);
opti.subject_to(x_1 - x_C_f + B1*M >= phi*v_C_f + delta);
opti.subject_to(x_C_f - x_2 + (1-B2)*M >= phi*v_2 + delta);
opti.subject_to(x_2 - x_C_f + B2*M >= phi*v_C_f + delta);
opti.subject_to(x_C_f - x_3 + (1-B3)*M >= phi*v_3 + delta);
opti.subject_to(x_3 - x_C_f + B3*M >= phi*v_C_f + delta);

[p_u_1, p_l_1] = reachable_set_p(x_1,x_1_0,v_1,v_1_0,tf);
[p_u_2, p_l_2] = reachable_set_p(x_2,x_2_0,v_2,v_2_0,tf);
[p_u_3, p_l_3] = reachable_set_p(x_3,x_3_0,v_3,v_3_0,tf);

opti.subject_to(p_u_1 <= 0);
opti.subject_to(p_l_1 >= 0);
opti.subject_to(p_u_2 <= 0);
opti.subject_to(p_l_2 >= 0);
opti.subject_to(p_u_3 <= 0);
opti.subject_to(p_l_3 >= 0);
opti.subject_to(v_min<=v_1<=v_max);
opti.subject_to(v_min<=v_2<=v_max);
opti.subject_to(v_min<=v_3<=v_max);

%% Solver Parameter Options
solver_options = struct('print_time', 1, 'discrete',discrete);
opti.solver('bonmin',solver_options); % set numerical backend
% opti.solver('bonmin',struct('print_time',1,'bonmin',...
%     struct('max_iter',100000))); % set numerical backend
sol = opti.solve();   % actual solve

sol.value(B1)
sol.value(B2)
sol.value(B3)


function [p_u_i,p_l_i] = reachable_set_p(x_i,x_i_0,v_i,v_i_0,t)
u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
u_min = -7; 
nu = (u_max+u_min)/2;
mu = (u_max-u_min)/2;
p_u_i = -0.5*t^2 + 0.25*((v_i-v_i_0-nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/nu;
p_l_i = 0.5*t^2 - 0.25*((-v_i+v_i_0+nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/mu;
end

function [gamma_x, gamma_v] = compute_disruption_norm_cons...
    (gamma, v_max, v_min, v_0, v_d, t_avg)
    gamma_x = gamma/(max(v_max-v_0,v_min-v_0)*t_avg)^2;
    gamma_v = (1-gamma)*max(v_max-v_d, v_min-v_d)^2;
end
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
% Preallocate vehicle array
X_0 = zeros(5,2);
% Vehicle U initial States;
v_U_0 = 20;
x_U_0 = 100;
% Vehicle C States;
v_C_0 = 24;
x_C_0 = 30;
% Vehicle 1 States;
v_1_0 = 28;
x_1_0 = 115;
X_0(1,:) = [x_1_0, v_1_0];
% Vehicle 2 States;
v_2_0 = 28;
x_2_0 = 85;
X_0(2,:) = [x_2_0, v_2_0];
% Vehicle 3 States;
v_3_0 = 24;
x_3_0 = 60;
X_0(3,:) = [x_3_0, v_3_0];
% Vehicle 4 States;
v_4_0 = 24;
x_4_0 = 25;
X_0(4,:) = [x_4_0, v_4_0];
% Vehicle 5 States;
v_5_0 = 24;
x_5_0 = 0;
X_0(5,:) = [x_5_0, v_5_0];


% Solve  MINLP
[tf, x_e_f, v_e_f, x_f, v_f, B, i_m] = ...
define_terminal_conditions (x_C_0, v_C_0, X_0(:,1), X_0(:,2), x_U_0, v_U_0);

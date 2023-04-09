clear all
clc

syms vf vl Fr D M u tau

x_states = [vf D];

f = [-Fr/M
    vl - vf];
g = [1/M
    0];
     
cbf = tau*vf - D;

Lfb = simplify(jacobian(cbf, x_states))*f;
Lgb = simplify(jacobian(cbf, x_states))*g;






% Lfb = simplify(jacobian(distance, x))*f
% 
% Lf2b = simplify(jacobian(Lfb, x))*f
% 
% LgLfb = simplify(jacobian(Lfb, x))*g
% 
% Ob = simplify(jacobian(p1*distance^q1,x))*f




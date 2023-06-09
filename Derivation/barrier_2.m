clear all
clc

syms px py v theta xo yo r p1 p2 u1 u2 real

x_states = [px; py;v; theta];
u = [u1,u2]';
f = [v * cos(theta);
     v * sin(theta);
     0;
    0];
g = [L;
    eye(2)];
     
distance = (px - xo)^2 + (py - yo)^2 - r^2;
% derivDistance = 2*(px-xo)*v*cos(theta) + 2*(py-yo)*v*sin(theta);
cbf = distance;
Lfb = simplify(jacobian(cbf, x_states))*f;
% Lgb = simplify(jacobian(cbf, x_states))*g;
LgLfb = simplify(jacobian(Lfb, x_states))*g;
Lf2b = simplify(jacobian(Lfb, x_states))*f;





% Lfb = simplify(jacobian(distance, x))*f
% 
% Lf2b = simplify(jacobian(Lfb, x))*f
% 
% LgLfb = simplify(jacobian(Lfb, x))*g
% 
% Ob = simplify(jacobian(p1*distance^q1,x))*f




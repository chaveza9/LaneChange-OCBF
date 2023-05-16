clear all
clc

syms px py v theta xo yo phi p1 Lw u1 u2 theta_max real  

x_states = [px; py;theta;v;phi];
u = [u1,u2]';
f = [v * cos(theta);
     v * sin(theta);
     v*tan(phi)/Lw
     0;
     0];
g = [zeros(3,2);
    eye(2)];
     
distance = (px - xo)^2;
deriv = 2*(px-xo)*v*cos(theta);
cbf = deriv+p1*distance; % position cbf
cbf = distance; % position cbf
% cbf  =  (2*theta*v*tan(phi))/Lw+p1*theta^2; %heading cbf
% cbf  = (theta-theta_max)^2;
cbf = v*sin(theta)*(2*py - 2*yo)+p1*(py-yo)^2;
Lfb = simplify(jacobian(cbf, x_states))*f;
Lgb = simplify(jacobian(cbf, x_states))*g;





% Lfb = simplify(jacobian(distance, x))*f
% 
% Lf2b = simplify(jacobian(Lfb, x))*f
% 
% LgLfb = simplify(jacobian(Lfb, x))*g
% 
% Ob = simplify(jacobian(p1*distance^q1,x))*f




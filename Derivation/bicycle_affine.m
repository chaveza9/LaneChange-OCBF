clear all
clc

syms px py v theta xo yo phi p1 Lw u1 u2 theta_max w1 w2 w3 w4 vd real  

x_states = [px; py;v;theta];
u = [u1,u2]';
w = [w1, w2, w3, w4]';
f = [v * cos(theta);
     v * sin(theta);
     0 ;
     0 ];
g = [0 -v*sin(theta);
     0 v*cos(theta);
     1 0; 
     0 v/Lw];
h = eye(4)*w;
 
distance = (px - xo)^2;
deriv = 2*(px-xo)*v*cos(theta);
cbf = deriv+p1*distance; % position cbf
% cbf  =  (2*theta*v*tan(phi))/Lw+p1*theta^2; %heading cbf
% cbf  = (theta)^2;
% cbf = (px - xo)^2;
% cbf = v*sin(theta)*(2*py - 2*yo)+p1*(py-yo)^2;
%cbf = (v-vd)^2;
Lfb = simplify(jacobian(cbf, x_states))*f;
Lhb = simplify(jacobian(cbf, x_states))*h;
Lgb = simplify(jacobian(cbf, x_states))*g;





% Lfb = simplify(jacobian(distance, x))*f
% 
% Lf2b = simplify(jacobian(Lfb, x))*f
% 
% LgLfb = simplify(jacobian(Lfb, x))*g
% 
% Ob = simplify(jacobian(p1*distance^q1,x))*f




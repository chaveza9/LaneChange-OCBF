clear all
clc

syms x_r y_r v_r theta_r  u1_r u2_r R_min real
syms x_h y_h v_h theta_h  u1_h u2_h real
syms delta_x delta_y delta_v delta_theta  u1_h u2_h real

x_states = [delta_x; delta_y;delta_v; delta_theta];

u = [u1_r-u1_h,u2_r-u2_h]';
f_r = [v_r * cos(theta_r);
     v_r * sin(theta_r);
     0;
     0];

f_h = [v_h * cos(theta_h);
     v_h * sin(theta_h);
     0;
     0];
g = [zeros(2);
    eye(2)];
     
cbf = (delta_x)^2 + (delta_y)^2 - R_min^2;
    

Lfb = simplify(jacobian(cbf, x_states))*(f_r-f_h);

Lf2b = simplify(jacobian(Lfb, x_states))*(f_r-f_h);




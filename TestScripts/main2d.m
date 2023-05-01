clear all
close all
clc
if ismac
    addpath('./casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('.\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*


% Initial Conditions
x0 = [5.0, 25.0, 0.0]';
% Buffer history
x_history = [];
t = 0;
dt = 0.1;
t0 = 0;
T  = 50;
end_sim = 0;
while ~end_sim
    % Append History
    x_history= cat(2,x_history,x0);
    % Compute control action
    u = clf_cbf(x0);
    % Integrate forward
    t_end_t = min(t + dt, t0+T);
    [ts_t, xs_t] = ode45(@(t, x) dynamics(t, x, u), ...
            [t, t_end_t], x0);
    t = ts_t(end);
    x0 = xs_t(end, :)'+[1;1;0]*randn(1)*0.3;
    if t_end_t == t0+T
        end_sim = 1;
    end
%     x0 = integrate_forward(x0, u);
    % Append history
end

% Plot Actions
x_obst = [32.0, 25.0];
r = [7];
figure
h = animatedline('MaximumNumPoints', length(x_history(1,:)), ...
    'Color',"blue","LineStyle","--");
for k=1:length(x_history(1,:))
    addpoints(h,x_history(1, k), x_history(2, k)); hold on;%grid minor
    for i = 1:size(x_obst, 1)
        draw_circle(x_obst(i, :), r(i));
    end
    % plot(x_history(1, k), x_history(2, k),'o','MarkerFaceColor','red');
    % hold off
    lim_min = min(min(x_history(1, :)), min(x_history(2, :)));
    lim_max = max(max(x_history(1, :)), max(x_history(2, :)));
    lim_min = min([lim_min, x_obst(:, 1)'-r, x_obst(:, 2)'-r]);
    lim_max = max([lim_max, x_obst(:, 1)'+r, x_obst(:, 2)'+r]);
    xlim([lim_min, lim_max]);
    ylim([lim_min, lim_max]);
    xlabel('x [m]')
    ylabel('y [m]')
    drawnow 
    axis equal
    Frames(k) = getframe(gcf);
end
writerObj = VideoWriter('result','MPEG-4');
writerObj.FrameRate = round(k/t_end_t);
open(writerObj)
writeVideo(writerObj, Frames);
close(writerObj);
function h = draw_circle(center,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + center(1);
    yunit = r * sin(th) + center(2);
    h = plot(xunit, yunit, 'Color',"#EDB120");
    
end


%% Define Phyisical Constraints
function xdot = dynamics(t, x,u)
    % compute derivatives
    theta = x(3);
    v = 1;
    xdot = [v * cos(theta);
        v * sin(theta);
        u];
end
function x = integrate_forward(x_0, u)
    substeps = 1;
    dt = 0.1;
%     # Solve the MPC problem to get the next state
    % Integrate forward
    x = x_0;
    xdot = dynamics(x,u)';
    for i=1:substeps
        x = x +  xdot*dt/substeps;
    end
end
function [u] = clf_cbf(x)
    opti = casadi.Opti(); % Optimization problem
    %% Problem Parameters
    x_goal = [45.0, 25.0];
    x_obst = [32.0, 25.0
              -10, -10];
    r = [7,7];
    %% Dynamics Parameters
    
    n_controls = 1;
    n_clf = 2;
    n_cbf = size(x_obst,1);
    n_cbf_clf = n_cbf + n_clf;
    control_bounds = [0.5]';
    
    %% Setup optimization variables
    u = opti.variable(n_controls,1); % control variables
    slack = opti.variable(n_cbf,1);
    delta = opti.variable(n_controls,1); % slack variables
    
    %% Decompose States
    px = x(1); py = x(2); v = 1; theta = x(3);
    % Compute conditions CBF
    
    cbf_gamma0 = 0.1; cbf_rate = 5;
    clf_rate = 0.5;
    weight_slack_cbfs = 100 * ones(size(r));
    weight_slack_clf = 1;
    weight_slack = [weight_slack_clf, weight_slack_cbfs];
    %CLF
    V_angle = (cos(theta)*(py-x_goal(2))-sin(theta)*(px-x_goal(1)))^2;
    V = V_angle;
    LgV = -2*(cos(theta)*(px - x_goal(1)) + sin(theta)*...
            (py - x_goal(2)))*(cos(theta)*(py - x_goal(2)) - ...
            sin(theta)*(px - x_goal(1)));
    LfV = 0;
    opti.subject_to(LgV.*u + clf_rate*V <= delta)
    % CBF
    for i = 1:size(x_obst,1)
        Lfb = 2*cos(theta)*(4*cos(theta) + cbf_gamma0*(2*px - 2*x_obst(i,1))) + 2*sin(theta)*(4*sin(theta) + cbf_gamma0*(2*py - 2*x_obst(i,2)));
        Lgb = cos(theta)*(4*py - 4*x_obst(i,2)) - sin(theta)*(4*px - 4*x_obst(i,1));
        distance = (px - x_obst(i,1))^2 + (py - x_obst(i,2))^2 - r(i)^2;
        derivDistance = 2*(px-x_obst(i,1))*v*cos(theta) + 2*(py-x_obst(i,2))*v*sin(theta);
        cbf = derivDistance + cbf_gamma0*distance;
        if Lgb
            opti.subject_to(Lfb+Lgb*u+cbf_rate*cbf>=-slack(i)) %cbf1
        end
    end

    % Add Actuation Limits
    opti.subject_to(-control_bounds<=u<=control_bounds)
   
    U = [u;delta;slack];
    % Define objective
    H = [eye(n_controls), zeros(n_controls,length(U)-n_controls);
        zeros(length(U)-n_controls,n_controls), diag(weight_slack)];
    objective = 0.5*U'*H*U;
    % Solve
    opti.minimize(objective)
   
    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
    sol = opti.solve();   % actual solve
    u = sol.value(u);

end
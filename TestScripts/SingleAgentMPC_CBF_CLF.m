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
x0 = [-2.0, 0.0, 0.0, 0.0]';
% Time parameters
dt = 0.1; % step size [s]
H = 3; % horizon time [s]
t0 = 0; %initial time [s]
T  = 20; % terminal time [s]
substep = 10; % integration substeps
% Buffer history
x_history = [];
% Define problem
x_goal = [2.5, 0.001, 0.5, 0.0];
x_obst = [0, 0];
r = [0.5, 7];
% util variables
end_sim = 0; % boolean for end simulation
t = 0; % current sim time
n_infeasible = 0;
x_guess = nan;
u_guess = nan;
% Define a progress bar
w = waitbar(0,'Initializing problem');
% Define mpc problem
[opti, x0_var, u0_var, u_var, x_var] = define_mpc(x_obst, r, x_goal, H, dt);
while ~end_sim
    % Append History
    x_history= cat(2,x_history,x0);
    % Solve MPC CBF-CLF
    [status, u0, x_guess, u_guess] = solve_mpc(opti.copy, ...
        x0_var, u0_var, u_var, x_var, x0, u_guess, x_guess);
    if ~status
        n_infeasible = n_infeasible +1;  
    end
    % Integrate forward
    t_end_t = min(t + dt, t0+T);
    if t_end_t == t0+T || norm(x0-x_goal,2)<=0.5
        end_sim = 1;
    end
    t = t_end_t;
    x0 = integrate_forward(t, x0, u0, dt, substep);
    % update progress bar
    msg = sprintf('Computing MPC solution at time %.2f/%.2f  \n',t,T);
    waitbar(t/T, w, msg);
end
opti.delete;
% Plot Actions
fig = create_history_plot(x_history, x_obst, r);

function fig = create_history_plot(x_history, x_obst, r)
    fig = figure;
    plot(x_history(1, :), x_history(2, :)); hold on;
    for i = 1:size(x_obst, 1)
        draw_circle(x_obst(i, :), r(i));
    end
    lim_min = min(min(x_history(1, :)), min(x_history(2, :)));
    lim_max = max(max(x_history(1, :)), max(x_history(2, :)));
    lim_min = min([lim_min, x_obst(:, 1)'-r, x_obst(:, 2)'-r]);
    lim_max = max([lim_max, x_obst(:, 1)'+r, x_obst(:, 2)'+r]);
    xlim([lim_min, lim_max]);
    ylim([lim_min, lim_max]);
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
end

function h = draw_circle(center,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + center(1);
    yunit = r * sin(th) + center(2);
    h = plot(xunit, yunit);
    hold off
end


%% Define Phyisical Constraints
function x = integrate_forward(t, x_0, u, dt, substeps)
    % Solve the MPC problem to get the next state
    % Integrate forward
    x = x_0;
    xdot = dynamics(t, x,u);
    for i=1:substeps
        x = x +  xdot*dt/substeps;
    end
end
function x_dot = dynamics(~, x, u)
    % compute derivatives
    x_dot = f(x)+g(x)*u;
end

function x_dot = f(x)
    x_dot = [x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
end

function val = g(~)
    val = [zeros(2,2);
         eye(2)];
end


function [fun] = compute_lie_derivative_2nd_order(barrier_fun, alpha_fun_1, alpha_fun_2)
    %% Define CBF functions
    n_states = 4;
    x_s = casadi.MX.sym('x',n_states);
    %% Compute cbf constraints
    alpha_0 = barrier_fun(x_s);
    b = casadi.Function('alpha0',{x_s},...
              {alpha_0});
    % Compute constraint parameters 
    db_dx = b.jacobian_old(0,0);
    Lfb = casadi.Function('Lfb',{x_s},...
              {db_dx(x_s)*f(x_s)});
    db2_dx = Lfb.jacobian_old(0,0);
    Lf2b = casadi.Function('Lf2b',{x_s},...
              {db2_dx(x_s)*f(x_s)});
    LgLfb = casadi.Function('LgLfb',{x_s},...
              {db2_dx(x_s)*g(x_s)});
    % Compute HO terms
    psi0 = b(x_s);
    psi1 = casadi.Function('psi_1',{x_s},...
              {Lfb(x_s)+alpha_fun_1(psi0)});
    psi1_dot = psi1.jacobian_old(0,0);
    
    psi2 = casadi.Function('psi_1',{x_s},...
              {psi1_dot(x_s)*f(x_s)+alpha_fun_2(psi1(x_s))});

    % construct constraints
    A = (LgLfb(x_s));
    B = (Lf2b(x_s) + psi2(x_s));
    fun = casadi.Function('constraint_fun', {x_s},{A,B});
end
function [fun] = compute_lie_derivative_1st_order(barrier_fun, alpha_fun_1)

    %% Define CBF functions
    n_states = 4;
    x_s = casadi.MX.sym('x',n_states);
    %% Compute cbf constraints
    alpha_0 = barrier_fun(x_s);
    b = casadi.Function('alpha0',{x_s},...
              {alpha_0});
    % Compute constraint parameters 
    db_dx = b.jacobian_old(0,0);
    Lfb = casadi.Function('Lfb',{x_s},...
              {db_dx(x_s)*f(x_s)});
    Lgb = casadi.Function('LgLfb',{x_s},...
              {db_dx(x_s)*g(x_s)});
    % Compute HO terms
    psi0 = b(x_s);
    psi1 = casadi.Function('psi_1',{x_s},...
              {Lfb(x_s)+alpha_fun_1(psi0)});

    % construct constraints
    A = (Lgb(x_s));
    B = (Lfb(x_s) + psi1(x_s));   
    fun = casadi.Function('constraint_fun', {x_s},{A,B});
end

function [opti, x0, u0, u, x] = define_mpc(x_obst, r, x_goal, h, dt)
    % ------------------------------------
    % x_0 = initial conditions [4,1]
    % h = horizon time [s]
    % dt = step time [s]
    % ------------------------------------
    opti = casadi.Opti(); % Optimization problem
    %% CBF-CLF Parameters
    n_controls = 2;
    n_states = 4;
    n_clf = 2;
    n_cbf = size(x_obst,1)+2;

    %% Define Physical Constraints
    control_bounds = [1, 0.5]';

    %% Define Simulation Parameters
    N = floor(h/dt);

    %% Setup optimization variables
    % Dynamics Variables
    u = opti.variable(n_controls, N); % control variables
    x = opti.variable(n_states, N+1); % control variables
    % Slack Variables
    slack_clf = opti.variable(n_controls,N);
    slack_cbf = opti.variable(size(x_obst,1),N);

    %% Define Lie Derivative Functions
    % Tunning rates
    p1 = 1; p2 = 50;
    clf_rate = 1;
    weight_slack_cbfs = 10000 * ones(size(x_obst,1));
    weight_slack_clf = [1,1];
    weight_slack = [weight_slack_clf, weight_slack_cbfs];   

    % ----------  Compute conditions CLF  ---------- 
    % define Lyapunov functions
    V_speed = @(x) (x(3)-x_goal(3))^2;
    V_angle = @(x) (cos(x(4))*(x(2)-x_goal(2))-sin(x(4))*(x(1)-x_goal(1)))^2;
    V = {V_speed, V_angle};
    V_fun = cell(n_clf,1);
    for i =1:length(V)
        % Define lyapunov function
        v_i = V{i};
        % Define alpha function
        alpha_1 = @(psi) clf_rate*psi;
        % Compute clf constraints
        V_fun{i} = compute_lie_derivative_1st_order(v_i, alpha_1);
    end

    % ----------  Compute conditions CBF ---------- 

    % define barrierfunctions
    v_min = @(x) x(3)-0.1;
    v_max = @(x) 2 - x(3);
    B = {};
    % Concatenate barrier functions in accordance to number of obstacles
    for i =1:size(x_obst,1)
        dist_i = @(x) (x(1) - x_obst(i,1))^2 + (x(2) - x_obst(i,2))^2 - r(i)^2;
        B{i} = dist_i;
    end
    B = cat(2,B,{v_min,v_max});
    B_fun = cell(1,n_cbf);
    for i =1:length(B)
        % Define barrier function
        b_i = B{i};
        if i ==1 || i ==2 % Distance CBF
            % Define alpha functions
            alpha_1 = @(psi) p1*psi;
            alpha_2 = @(psi) p2*psi;
            % Compute CBF constraints
            B_fun{i} = compute_lie_derivative_2nd_order(...
                b_i, alpha_1, alpha_2);
        else
            % Define alpha functions
            alpha_1 = @(psi) p1*psi;
            % Compute CBF constraints
            B_fun{i} = compute_lie_derivative_1st_order(b_i, alpha_1);
        end
    end

    %% Define Constraints
    % initialize objective 
    objective = 0; 
    % iterate through time
    
    for k = 1:N
        % ---------- Compute Dynamics constraints ---------
        dxi_dt = dynamics(k,x(:,k),u(:,k));
        xi_next = x(:,k+1);
        opti.subject_to(xi_next == x(:,k) + dt * dxi_dt)
        
        % -------- Form CLF constraint matrix --------
        % define buffer for constraints
        A_clf =  casadi.MX.zeros(n_clf, n_controls);
        b_clf = casadi.MX.zeros(n_clf, 1);
        for i = 1:length(V_fun)
            [G, F] = V_fun{i}(xi_next);
            A_clf(i,:) = G;
            b_clf(i) = -(F);
        end
        opti.subject_to(A_clf * u(:,k) <= b_clf+slack_clf(:,k))
        
        % -------- Form CBF constraint matrix --------
        % Define buffer for constraints
        A_cbf =  casadi.MX.zeros(n_cbf, n_controls);
        b_cbf = casadi.MX.zeros(n_cbf, 1);
        for i = 1:length(B_fun)
            [G, F] = B_fun{i}(xi_next);
            A_cbf(i,:) = -G;
            b_cbf(i) = F;
        end
        opti.subject_to(A_cbf * u(:,k) <= b_cbf+[slack_cbf(:,k);zeros(2,1)])
        
        % ----------- Compute Objective Function -----------------
        % Add Slack Variables
        U = [u(:,k);slack_clf(:,k);slack_cbf(:,k)];
        % Define Objective
        H = [eye(n_controls), zeros(n_controls,length(U)-n_controls);
            zeros(length(U)-n_controls,n_controls), diag(weight_slack)];        
        objective = objective + 0.5*U'*H*U;
    end
    
    % Add Actuation Limits
    opti.subject_to(-control_bounds <= u <=control_bounds)
    % Define Initial Conditions
    
    opti.minimize(objective)
    x0 = x(:,1);
    u0 = u(:,1);
end

function [status, u0, x, u] = solve_mpc(opti, x0_var, u0_var,... 
         u_var, x_var, curr_state, u_guess, x_guess)
    arguments
        opti
        x0_var
        u0_var
        u_var 
        x_var 
        curr_state
        u_guess = nan
        x_guess = nan
    end
    % define initial condition
    opti.subject_to(x0_var == curr_state)
    % define initial guess if available
    if ~isnan(x_guess)
        opti.set_initial(x_var, x_guess)
    end
    if ~isnan(u_guess)
        opti.set_initial(u_var, u_guess)
    end
    % Define optimizer settings
    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
    % Create solver and solve!
    try
        solution = opti.solve();
    catch
        status = false;
        u0 = zeros(size(u0_var));
        x = NaN;
        u = NaN;
        return
    end

    % Populate return values
    status = solution.stats.success;
    u0 = solution.value(u0_var);
    x = solution.value(x_var);
    u = solution.value(u_var);   
end
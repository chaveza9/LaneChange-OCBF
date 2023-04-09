clear all
close all
clc
if ismac
    addpath('../casadi-osx-matlabR2015a-v3.5.5')
else
    addpath('..\casadi-windows-matlabR2016a-v3.5.5')
end
import casadi.*


% Initial Conditions
x0 = [-2.0, 0.0, 0.0, 0.0]';
% Time parameters
dt = 0.1; % step size [s]
H = 3; % horizon time [s]
t0 = 0; %initial time [s]
T  = 10; % terminal time [s]
substep = 10; % integration substeps
% Buffer history
x_history = [];
% Define problem
x_goal = [1, 0.001, 0.5, 0.0];
x_obst = [0, 0];
r = [0.5];
% util variables
end_sim = 0; % boolean for end simulation
t = 0; % current sim time
n_infeasible = 0;
x_guess = nan;
u_guess = nan;
% Define a progress bar
w = waitbar(0,'Initializing problem');
%% Define mpc problem
% Define running cost
running_cost_fn = @(x,u) lqr_running_cost(x, u, x_goal, ...
    dt * diag([1.0, 1.0, 0.1, 0.0]), 0.01 * eye(2));
% define terminal cost
terminal_cost_fn = @(x) squared_error_terminal_cost(x, x_goal);
% define mpc object
[opti, x0_var, u0_var, u_var, x_var] = define_mpc(x_obst, r, H, dt, ...
    running_cost_fn, terminal_cost_fn);
%% Solve MPC Problem
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
    msg = sprintf('Computing MPC solution at time %.2f s/%.2f s  \n',t,T);
    waitbar(t/T, w, msg);
end
opti.delete;
close(w)
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
    grid minor
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

function cost = lqr_running_cost(x, u, x_goal, Q, R)
    % Returns the LQR running cost.
    %    
    %argument
    %    x: the current state
    %    u: the current control input
    %    x_goal: the goal state
    %    Q: the state cost weight matrix
    %    R: the control cost weight matrix
    %returns:
    %    (x - x_goal)^T Q (x - x_goal) + u^T R u
    % ---------------------------------------------------------------------
    arguments
        x {mustBeUnderlyingType(x, 'casadi.MX')}
        u {mustBeUnderlyingType(u, 'casadi.MX')}
        x_goal {mustBeVector}
        Q 
        R
    end
    x_goal = reshape(x_goal, size(x));
    cost =  (x - x_goal)'*Q* (x - x_goal) +  u'*R* u;
end

function cost = squared_error_terminal_cost(x, x_goal, Q)
    % Returns the LQR running cost.
    %    
    %argument
    %    x: the current state
    %    u: the current control input
    %    x_goal: the goal state
    %    Q: the state cost weight matrix
    %    R: the control cost weight matrix
    %returns:
    %    (x - x_goal)^T Q (x - x_goal) + u^T R u
    % ---------------------------------------------------------------------
    arguments
        x {mustBeUnderlyingType(x, 'casadi.MX')}
        x_goal {mustBeVector}
        Q = eye(length(x))
    end
    x_goal = reshape(x_goal, size(x));    
    cost =   (x - x_goal)'*Q* (x - x_goal);
end


function [opti, x0, u0, u, x] = define_mpc(x_obst, r, h, dt, ...
    running_cost_fn, terminal_cost_fn)
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

    %% Define Lie Derivative Functions
    % Tunning rates
    p1 = 1; p2 = 50;

    % ----------  Compute conditions CBF ---------- 
    % define barrierfunctions
    v_min = @(x) x(3)-0.01;
    v_max = @(x) 2 - x(3);
    B = {v_min,v_max};
    % Concatenate barrier functions in accordance to number of obstacles
    for i =1:size(x_obst,1)
        dist_i = @(x) (x(1) - x_obst(i,1))^2 + (x(2) - x_obst(i,2))^2 - r(i)^2;
        B = cat(2,B,{dist_i});
    end
    B_fun = cell(1,n_cbf);
    for i =1:length(B)
        % Define barrier function
        b_i = B{i};
        if i >2  % Distance CBF
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
    for k = 1:N
        % ---------- Compute Dynamics constraints ---------
        dxi_dt = dynamics(k,x(:,k),u(:,k));
        xi_next = x(:,k+1);
        opti.subject_to(xi_next == x(:,k) + dt * dxi_dt)
        
        % -------- Form CBF constraint matrix --------
        % Define buffer for constraints
        A_cbf =  casadi.MX.zeros(n_cbf, n_controls);
        b_cbf = casadi.MX.zeros(n_cbf, 1);
        for i = 1:length(B_fun)
            [G, F] = B_fun{i}(xi_next);
            A_cbf(i,:) = -G;
            b_cbf(i) = F;
        end
        opti.subject_to(A_cbf * u(:,k) <= b_cbf)        
    end
    
    % Add Actuation Limits
    opti.subject_to(-control_bounds <= u <=control_bounds)
    
    %% Define Cost
    % Create the objective and add it to the problem
    cost = terminal_cost_fn(x(:, end));
    for t =1:N
        cost = cost + running_cost_fn(x(:,k), u(:,k));
    end
    opti.minimize(cost)
    %% Define output variables
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
        warning('infeasible problem detected')
        return
    end

    % Populate return values
    status = solution.stats.success;
    u0 = solution.value(u0_var);
    x = solution.value(x_var);
    u = solution.value(u_var);   
end
        % CBF with 1 obstacle constraint
function [status, u] = solve_fxtm_cbf_1(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des)

    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; zeros(2,1)]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
    % determine if x_des is feasible
    if (x_ego(1)-x_des)^2<=10
        h_des_x = 0;
    else
        h_des_x = 1;
    end
    % Num constraints
    n_clf = 1+h_des_x; % Desired Speed, Desired Terminal Position
    n_cbf = 3; % speed constraints, Front vehicle ACC
    %% Define Relaxation variables
    slack_clf = opti.variable(n_clf,1); % control variables 
    slack_cbf = opti.variable(n_cbf,1); % control variables 
    %% Define Fixed Time constraints
    % alpha_i = opti.variable(n_clf,2); % time constraints
    % mu = opti.variable(n_clf,1); % tunning value
    % opti.subject_to(slack_clf<=-0.1)
    % Compute Fixed time guarantees rates
    gamma_1 = 1 + 1/self.mu_clf;
    gamma_2 = 1 - 1/self.mu_clf;
    
    alpha = self.mu_clf*pi/(2*t_f);
    % ----------  Compute conditions CLF ----------
    % Avoid nullifying desired speed)
    if (x_ego(2) - v_des)^2<=0.1
        v_des = v_des+0.1;
    end
    % Define Lyapunov Function
    h_speed = @(x) (x(2) - v_des)^2;
    p1 = 10;
    h_pos = @(x) 2*(x_des-x(1))*x(2)+p1*(x(1) - x_des)^2;
    % Concatenate functions
    if h_des_x
        h_goal = {h_pos, h_speed}; 
    else
        h_goal = {h_speed}; 
    end
    % Define qp matrix
    U = [u_var;zeros(2,1)];
    for i =1:length(h_goal)
        % Compute gamma exponents
        % alpha_i = mu(i)*pi/(2*t_f);
        % gamma_1 = 1+1/mu;
        % gamma_2 = 1-1/mu;
        opti.subject_to()
        % Define lyapunov function
        h_g_i = h_goal{i};
        % Compute clf constraints
        [Lgh_g, Lfh_g] = self.compute_lie_derivative_1st_order(h_g_i);
        opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) <= ...
            slack_clf(i)- alpha*max(0,h_g_i(x_p))^gamma_1 -...
            alpha*max(0,h_g_i(x_p))^gamma_2);                
    end

    % ----------  Compute conditions CBF ---------- 
    % define barrierfunctions
    b_v_min = @(x) (x(2)-self.velMin);
    b_v_max = @(x) (self.velMax - x(2));
    b_dist_ego_front = @(x) (x(3)-x(1))-self.tau*x(2)-self.delta_dist;
    h_safe = {b_v_min,b_v_max, b_dist_ego_front};            
    for i =1:length(h_safe)
        % Define barrier function
        h_s_i = h_safe{i};
        % Compute CBF constraints
        [Lgh_s, Lfh_s] = self.compute_lie_derivative_1st_order(h_s_i);
        opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U +slack_cbf(i)*h_s_i(x_p)>=0)
    end
    % Add safe slacks
    opti.subject_to(slack_cbf>=0.1);
    % Add Actuation Limits
    opti.subject_to(u_var>= self.accelMin)
    opti.subject_to(u_var<= self.accelMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_clf; slack_cbf];
    z_var(1:self.n_controls) = z_var(1:self.n_controls) - u_ref;
    % Create quadratic cost
    % normalizing control
    gamma_u = 1/max((self.accelMax-u_ref)^2,(self.accelMin-u_ref)^2);
    H_u = gamma_u*eye(self.n_controls);
    H_delta_clf = 200 * eye(n_clf);
    H_delta_cbf = 2000 * eye(n_cbf);
    H = blkdiag(H_u, H_delta_clf, H_delta_cbf);
    % Linear Cost
    F = [zeros(1, self.n_controls), 200*ones(1,n_clf), zeros(1,n_cbf)];
    % Define Objective
    objective = 0.5*z_var'*H*z_var+F*z_var;
    opti.minimize(objective)
    % ----------  Create solver and solve! ---------- 
    opts = self.define_solver_options;
    % opti.solver('sqpmethod',opts)
    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
    try
        solution = opti.solve_limited();
    catch err
        warning(err.identifier,"%s", err.message)
        status = false;
        u = NaN;
        warning('infeasible problem detected')
        return
    end
    % Populate return values
    status = solution.stats.success;
    u = solution.value(u_var);  
    opti.delete()
end

        


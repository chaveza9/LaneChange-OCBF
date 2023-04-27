        % CBF with 1 obstacle constraint
function [status, u] = solve_fxtm_clf(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des)

    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; zeros(2,1)]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
    % determine if x_des is feasible
    if x_des-x_ego(1)<=5
        h_des_x = 0;
    else
        h_des_x = 1;
    end
    % Num constraints
    n_clf = 1+h_des_x; % Desired Speed, Desired Terminal Position
    %% Define Relaxation variables
    slack_clf = opti.variable(n_clf,1); % control variables 
    %% Define Fixed Time constraints
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
    p1 = 3;
    h_pos = @(x) 2*(x_des-x(1))*x(2)+p1*(x(1) - x_des)^2;
    % Concatenate functions
    if h_des_x
        h_goal = {h_speed, h_pos}; 
    else
        h_goal = {h_speed}; 
    end
    % Define qp matrix
    U = [u_var;zeros(2,1)];
    for i =1:length(h_goal)
        % Compute gamma exponents
        % opti.subject_to()
        % Define lyapunov function
        h_g_i = h_goal{i};
        % Compute clf constraints
        [Lgh_g, Lfh_g] = self.compute_lie_derivative_1st_order(h_g_i);
        if true
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) <= ...
                -slack_clf(i)*h_g_i(x_p)- alpha*max(0,h_g_i(x_p))^gamma_1 -...
                alpha*max(0,h_g_i(x_p))^gamma_2);                
        else
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) <= ...
                -slack_clf(i)*h_g_i(x_p));                
        end
    end

    % Add Actuation Limits
    opti.subject_to(u_var>= self.accelMin)
    opti.subject_to(u_var<= self.accelMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_clf];
    z_var(1:self.n_controls) = z_var(1:self.n_controls) - u_ref;
    % Create quadratic cost
    % normalizing control
    gamma_u = 1/max((self.accelMax-u_ref)^2,(self.accelMin-u_ref)^2);
    H_u = gamma_u*eye(self.n_controls);
    H_delta_clf = 2 * eye(n_clf);
    H = blkdiag(H_u, H_delta_clf);
    % Linear Cost
    F = [zeros(1, self.n_controls), 100*ones(1,n_clf)];
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
        u = u_ref;
        warning('infeasible problem detected')
        return
    end
    % Populate return values
    status = solution.stats.success;
    u = solution.value(u_var);  
    opti.delete()
end

        


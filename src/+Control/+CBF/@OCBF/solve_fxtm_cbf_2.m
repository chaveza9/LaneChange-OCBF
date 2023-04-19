 % CBF with 2 obstacle constraint (mergin constraint
function [status, u] = solve_fxtm_cbf_2(self, ...
    x_ego, x_front, u_ref, t_f, v_des, x_des, x_adj_front, phi)
    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; x_adj_front]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
    % Num constraints
    n_clf = 2; % Desired Speed, Desired Terminal Position
    n_cbf = 4; % speed constraints, Front vehicle ACC, adjacent veh ACC
    % Compute Fixed time guarantees rates
    gamma_1 = 1 + 1/self.mu_clf;
    gamma_2 = 1 - 1/self.mu_clf;
    alpha = self.mu_clf*pi/(2*t_f);
    %% Define Relaxation variables
    slack_clf = opti.variable(n_clf,1); % control variables 
    slack_cbf = opti.variable(n_cbf,1); % control variables 
    % ----------  Compute conditions CLF ----------
    % Define Lyapunov Function
    h_speed = @(x) (x(2) - v_des)^2;
    h_pos = @(x) (x(1) - x_des)^2;
    % Concatenate functions
    h_goal = {h_pos, h_speed}; 
    % Define qp matrix
    U = [u_var;zeros(2,1)];
    for i =1:length(h_goal)
        % Define lyapunov function
        h_g_i = h_goal{i};
        % Compute clf constraints
        [Lgh_g, Lfh_g] = self.compute_lie_derivative_1st_order(h_g_i);
        opti.subject_to(Lgh_g(x_p)*U - h_g_i(x_p)* slack_clf(i) <= ...
            - Lfh_g(x_p) - alpha*max(0,h_g_i(x_p))^gamma_1 -...
            alpha*max(0,h_g_i(x_p))^gamma_2);                
    end

    % ----------  Compute conditions CBF ---------- 
    % define barrierfunctions
    b_v_min = @(x) -(x(2)-self.velMin);
    b_v_max = @(x) -(self.velMax - x(2));
    b_dist_ego_front = @(x) self.tau*x(2) - (x(3)-x(1) + self.delta_dist);
    b_dist_ego_adj = @(x) phi*x(2) - (x(5)-x(1))  + self.delta_dist;
    h_safe = {b_v_min,b_v_max, b_dist_ego_front, b_dist_ego_adj};            
    for i =1:length(h_safe)
        % Define barrier function
        h_s_i = h_safe{i};
        % Compute CBF constraints
        [Lgh_s, Lfh_s] = self.compute_lie_derivative_1st_order(h_s_i);
        opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U <= -slack_cbf(i)*h_s_i(x_p))
    end
    
    % Add Actuation Limits
    opti.subject_to(u_var>=-self.accelMin)
    opti.subject_to(u_var<= self.accelMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_clf; slack_cbf];
    z_var(1:self.n_controls) = z_var(1:self.n_controls) - u_ref;
    % Create quadratic cost
    H_u = 2*eye(self.n_controls);
    H_delta_clf = 2 * eye(n_clf);
    H_delta_cbf = 200 * eye(n_cbf);
    H = blkdiag(H_u, H_delta_clf, H_delta_cbf);
    % Linear Cost
    F = [zeros(1, self.n_controls), 2*ones(1,n_clf), zeros(1,n_cbf)];
    % Define Objective
    objective = 0.5*z_var'*H*z_var+F*z_var;
    opti.minimize(objective)
    % ----------  Create solver and solve! ---------- 
    opts = self.define_solver_options;
    opti.solver('ipopt',opts)
    try
        solution = opti.solve_limited();
    catch err
        warning(err.identifier,"%s", err.message)
        status = false;
        u = NaN;
        warning(err.)
        return
    end
    % Populate return values
    u = solution.value(u_var);  
    opti.delete()
end
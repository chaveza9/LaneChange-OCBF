 % CBF with 2 obstacle constraint (mergin constraint
function [status, u] = solve_fxtm_clbf_2(self, ...
    x_ego, x_front, u_ref, t_f, v_des, x_des, x_adj_front, phi, theta_des)
    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; x_adj_front]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
     % determine if x_des is feasible
    if (x_ego(1)-x_des)^2<=10
        h_des_x = 0;
    else
        h_des_x = 1;
    end
    % Num constraints
    n_clf = 2+h_des_x; % Desired Speed, Desired Terminal Position
    n_cbf = 4; % speed constraints, Front vehicle ACC, adjacent veh ACC
    % Compute Fixed time guarantees rates
    gamma_1 = 1 + 1/self.mu_clf;
    gamma_2 = 1 - 1/self.mu_clf;
    alpha = self.mu_clf*pi/(2*t_f);
    %% Define Relaxation variables
    slack_clf = opti.variable(n_clf,1); % control variables 
    slack_cbf = opti.variable(n_cbf,1); % control variables 
    % ----------  Compute conditions CLF ----------
    % Avoid nullifying desired speed)
    if (x_ego(3) - v_des)^2<=0.1
        v_des = v_des+0.1;
    end
    % Define Lyapunov Function
    h_angle = @(x) (x(4) - theta_des)^2;
    h_speed = @(x) (x(3) - v_des)^2;
    p1 = 3;
    h_pos = @(x) 2*(x_des-x(1))*x(3)*cos(x(4))+p1*(x(1) - x_des)^2;
    % Concatenate functions
    if h_des_x
        h_goal = {h_angle, h_speed, h_pos}; 
    else
        h_goal = {h_angle, h_speed}; 
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
        if i<=2
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p)-slack_clf(i)<=...
                -h_g_i(x_p));                
        else
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) <= ...
                -slack_clf(i)*h_g_i(x_p)- alpha*max(0,h_g_i(x_p))^gamma_1 -...
                alpha*max(0,h_g_i(x_p))^gamma_2);                
        end
    end

    % ----------  Compute conditions CBF ---------- 
    % define barrierfunctions
    b_v_min = @(x) (x(3)-self.velMin);
    b_v_max = @(x) (self.velMax - x(3));
    b_dist_ego_front = @(x) (x(5)-x(1))-self.tau*x(3)-self.delta_dist;
    b_dist_ego_adj = @(x) (x(7)-x(1))-phi(x)*x(3) -self.delta_dist;
    h_safe = {b_v_min,b_v_max, ...
        b_dist_ego_front, b_dist_ego_adj};            
    for i =1:length(h_safe)
        % Define barrier function
        h_s_i = h_safe{i};
        % Compute CBF constraints
        [Lgh_s, Lfh_s] = self.compute_lie_derivative_1st_order(h_s_i);
        if false || i>=3
            opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U +slack_cbf(i)*h_s_i(x_p)^2>=0)
        else
            opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U +h_s_i(x_p)^2>=0)
        end
        
    end
    
    opti.subject_to(slack_cbf(i)>=0.0);
    opti.subject_to(slack_cbf(i)<=1/self.dt);
    
    % Add Actuation Limits
    opti.subject_to(u_var(1)>= self.accelMin)
    opti.subject_to(u_var(1)<= self.accelMax)
    opti.subject_to(u_var(2)>= self.omegaMin)
    opti.subject_to(u_var(2)<= self.omegaMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_clf; slack_cbf];
    z_var(1:self.n_controls-1) = z_var(1:self.n_controls-1) - u_ref;
    % Create quadratic cost
    % normalizing control
    gamma_u = 1/max((self.accelMax-u_ref)^2,(self.accelMin-u_ref)^2);
    gamma_omega = 1/max((self.omegaMax)^2,(self.omegaMin)^2);
    H_u = diag([gamma_u, gamma_omega]);
    if ~h_des_x
        H_delta_clf = diag([10,100]);
        F_slack_clf = [0, 0];
    else
        H_delta_clf = diag([10,10,10]);
        F_slack_clf = [0, 1000, 1000];
    end
    H_delta_cbf = diag([10,10,10,10]);
    H = blkdiag(H_u, H_delta_clf, H_delta_cbf);
    % Linear Cost
    F = [zeros(1, self.n_controls), F_slack_clf, zeros(1,n_cbf)];
    % Define Objective
    objective = 0.5*z_var'*H*z_var+F*z_var;
    opti.minimize(objective)
    % ----------  Create solver and solve! ---------- 
    % opts = self.define_solver_options;
    % opti.solver('sqpmethod',opts)
    opti.solver('ipopt',struct('print_time',0,'ipopt',...
    struct('max_iter',10000,'acceptable_tol',1e-8,'print_level',1,...
    'acceptable_obj_change_tol',1e-6))); % set numerical backend
    try
        solution = opti.solve_limited();
    catch err
        warning(err.identifier,"%s", err.message)
        status = false;
        u = [u_ref,0]';
        warning('infeasible problem detected')
        return
    end
    % Populate return values
    status = solution.stats.success;
    u = solution.value(u_var);  
    opti.delete()
end
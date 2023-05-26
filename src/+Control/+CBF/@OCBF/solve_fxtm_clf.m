        % CBF with 1 obstacle constraint
function [status, u] = solve_fxtm_clf(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des, y_des, flag)

    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; zeros(2,1)]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
    % determine if x_des is feasible
    if x_des-x_ego(1)<=15 || flag 
        h_des_x = 1;
        x_des = x_des+150;
        t_f = 10;
    else
        h_des_x = 1;
    end
    if contains(self.method, 'cbf') 
        h_des_x = 0;
    end
    skip_v_des = 0;
    if strcmp(self.method, 'cbf')
        skip_v_des = 1;
    end
    % Num constraints
    n_clf = 3+h_des_x; % Desired Speed, Desired Terminal Position
    %% Define Relaxation variables
    slack_clf = opti.variable(n_clf,1); % control variables 
    %% Define Fixed Time constraints
    % Compute Fixed time guarantees rates
    gamma_1 = 1 + 1/self.mu_clf;
    gamma_2 = 1 - 1/self.mu_clf;
    
    alpha = self.mu_clf*pi/(2*t_f);
    % ----------  Compute conditions CLF ----------
    % Avoid nullifying desired speed)
    if (x_ego(3) - v_des)^2<=0.1
        v_des = v_des+0.1;
    end
    % if (x_ego(2) - y_des)^2==0.0 
    %     y_des = y_des+0.001;
    % end
    % Define Lyapunov Function
    h_y = @(x) (x(2)-y_des)^2;
    h_angle = @(x) (x(4))^2;
    h_speed = @(x) (x(3) - v_des)^2;
    p1 = 3;
    h_pos = @(x) 2*(x_des-x(1))*x(3)*cos(x(4))+p1*(x(1) - x_des)^2;
    % Concatenate functions
    if h_des_x
        h_goal = {h_y, h_angle, h_speed, h_pos}; 
    else
        h_goal = {h_y, h_angle, h_speed}; 
    end
    % Define qp matrix
    U = [u_var;zeros(2,1)];
    %% Define dual variables
    lambda = opti.variable(n_clf,length(x_p)*2);
    %% Create constraints
    for i =1:length(h_goal)
        % Compute gamma exponents
        % opti.subject_to()
        % Define lyapunov function
        h_g_i = h_goal{i};
        % Compute clf constraints
        [Lgh_g, Lfh_g, lwh_g] = self.compute_lie_derivative_1st_order(h_g_i);
        % Extract dual variables
        lambda_i = lambda(i,:)';
        [b,A] = self.noise_lims;
        % Add clf constraint
        if i ==3 && skip_v_des
            continue
        end
        if i<=2 
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) + b'*lambda_i...
                -slack_clf(i)<= -h_g_i(x_p)); 
        else
            opti.subject_to(Lgh_g(x_p)*U + Lfh_g(x_p) + b'*lambda_i<= ...
                -slack_clf(i)*h_g_i(x_p)- alpha*max(0,h_g_i(x_p))^gamma_1 -...
                alpha*max(0,h_g_i(x_p))^gamma_2);                
        end
        % Add dual constraints
        opti.subject_to(lambda_i'*A==lwh_g(x_p))
        opti.subject_to(lambda_i>=0)
    end
    
    % Add Actuation Limits
    opti.subject_to(u_var(1)>= self.accelMin)
    opti.subject_to(u_var(1)<= self.accelMax)
    opti.subject_to(u_var(2)>= self.omegaMin)
    opti.subject_to(u_var(2)<= self.omegaMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_clf];
    z_var(1:self.n_controls-1) = z_var(1:self.n_controls-1) - u_ref;
    % Create quadratic cost
    % normalizing control
    gamma_u = 1/max((self.accelMax-u_ref)^2,(self.accelMin-u_ref)^2);
    gamma_omega = 1/max((self.omegaMax)^2,(self.omegaMin)^2);
    H_u = diag([gamma_u, gamma_omega]);
    if ~h_des_x 
        H_delta_clf = diag([10,10,10]);
        F_slack = [0, 0, 10]*0;
    elseif h_des_x && abs(x_ego(2)-y_des)>=0.25
        H_delta_clf = diag([10,10,10,10]);
        % if ~self.uncooperative
            F_slack = [50, 100, 300, 500];
        % else
        %     F_slack = [0, 100, 300, 200];
        % end
    else
        H_delta_clf = diag([10,10,10,50]);
        % if ~self.uncooperative
            F_slack = [0, 100, 300, 1100];
        % else
        %     F_slack = [0, 100, 100, 100];
        % end
    end


    H = blkdiag(H_u, H_delta_clf);
    
    % Linear Cost
    F = [zeros(1, self.n_controls), F_slack];
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
        u = [u_ref,0]';
        warning('infeasible problem detected')
        return
    end
    % Populate return values
    status = solution.stats.success;
    u = solution.value(u_var);  
    opti.delete()
end

        


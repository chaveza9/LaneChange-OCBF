        % CBF with 1 obstacle constraint
function [status, u] = solve_cbf_2(self, ...
            x_ego, x_front, u_ref, x_adj_front, phi)

    opti = casadi.Opti(); % Optimization problem
    %% Setup optimization variables
    % Optimization Variables
    u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
    % State Variables
    x_p = [x_ego; x_front; x_adj_front]; % state variables [x_ego, v_ego, x_obst, v_obst]
    
    %% CBF-CLF Parameters
    % Num constraints
    n_cbf = 4; % speed constraints, Front vehicle ACC, adjacent veh ACC
    %% Define Relaxation variables
    slack_cbf = opti.variable(n_cbf,1); % control variables 
    % Define qp matrix
    U = [u_var;zeros(2,1)];
    % ----------  Compute conditions CBF ---------- 
    % define barrierfunctions
    b_v_min = @(x) (x(3)-self.velMin);
    b_v_max = @(x) (self.velMax - x(3));
    b_dist_ego_front = @(x) (x(5)-x(1))-self.tau*x(3)-self.delta_dist;
    b_dist_ego_adj = @(x) (x(7)-x(1))-phi(x)*x(3) -self.delta_dist;
    h_safe = {b_v_min,b_v_max, ...b_theta_max...
        b_dist_ego_front, b_dist_ego_adj};            
    %% Define dual variables
    mu = opti.variable(n_cbf,length(x_p)*2);
    %% Create constraints
    for i =1:length(h_safe)
        % Define barrier function
        h_s_i = h_safe{i};
        % Compute CBF constraints
        [Lgh_s, Lfh_s, lwh_s] = self.compute_lie_derivative_1st_order(h_s_i);
        % Extract dual variables
        mu_i = mu(i,:)';
        [b,A] = self.noise_lims;
        % Compute constraint
        if i==3 || true
            opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U + b'*mu_i ...
                 +slack_cbf(i)*h_s_i(x_p)^2>=0)
        else
            opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U + b'*mu_i ...
                +h_s_i(x_p)^2>=0)
        end
        % Add dual constraints
        opti.subject_to(mu_i'*A==lwh_s(x_p))
        opti.subject_to(mu_i<=0)
    end
    
    opti.subject_to(slack_cbf(i)>=0.0);
    opti.subject_to(slack_cbf(i)<=1/self.dt);
    
    % Add Actuation Limits
    opti.subject_to(u_var(1)>= self.accelMin)
    opti.subject_to(u_var(1)<= self.accelMax)
    opti.subject_to(u_var(2)>= self.omegaMin)
    opti.subject_to(u_var(2)<= self.omegaMax)

    % ----------  Compute  qp objective ---------- 
    z_var = [u_var; slack_cbf];
    z_var(1:self.n_controls) = z_var(1:self.n_controls) - u_ref;
    % Create quadratic cost
    % normalizing control
    gamma_u = 1/max((self.accelMax-u_ref(1))^2,(self.accelMin-u_ref(1))^2);
    gamma_omega = 1/max((self.omegaMax)^2,(self.omegaMin)^2);
    H_u = diag([gamma_u, gamma_omega]);
    % H_delta_cbf = diag([10,10,10,10,10,10]);
    H_delta_cbf = diag([10,10,10,10]);
    H = blkdiag(H_u, H_delta_cbf);
    % Linear Cost
    F = [zeros(1, self.n_controls), zeros(1,n_cbf)];
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

        


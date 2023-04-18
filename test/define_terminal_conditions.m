function [tf, x_e_f, v_e_f, x_f, v_f, B, i_m] = define_terminal_conditions (x_C_0, v_C_0, x_0, v_0, x_U_0, v_U_0)
       % ------------------------------------
       % x_0 = initial conditions [numVehicles,1]
       % v_0 = initial conditions [numVehicles,1]
       % x_f, v_f = terminal conditions [numVehicles,1]
       % ------------------------------------
       arguments
           x_C_0
           v_C_0
           x_0
           v_0
           x_U_0
           v_U_0
       end
       opti = casadi.Opti(); % Optimization problem
       % Constraints
        u_max = 3.3;    % Vehicle i max acceleration [m/s^2]
        u_min = -3;   % Vehicle i min acceleration [m/s^2]
        v_min = 10;   % Vehicle i min velocity [m/s]
        v_max = 35;   % Vehicle i max velocity [m/s]
        T_max = 20;     % maximum allowable time
        t_avg = 10;      % Average time
        % Tunning Constants
        gamma_t = 1/T_max;
        gamma = 0.5;
        % Alpha cost
        alpha_t = 1;
        alpha_dis = 1;
        % Safety Parameters
        phi = 0.9;   % reaction time 
        delta = 15;  % Minimum safety distance
        v_d = 30;    % desired speed
        % Compute problem paramters
        numCandidates = size(x_0,1);
        M = 1000;
        %% Setup optimization variables
        % Dynamics Variables
        tf_var = opti.variable();

        X_ego = opti.variable(1, 2); % control variables
        x_ego_var = X_ego(:,1); % position cav c
        v_ego_var = X_ego(:,2); % position cav c

        X_target =  opti.variable(numCandidates, 2); % CAV states
        x_var = X_target(:,1); % position cavs
        v_var = X_target(:,2); % speed_cavs
        B_var = opti.variable(numCandidates, 1); % binary variables
        discrete = [zeros((numCandidates+1)*2+1,1);ones(numCandidates,1)];
        
        %% Define cost
        % Time cost
        cost_time = gamma_t*tf_var;
        % Disruption cost
        % CAV C
        [gamma_x, gamma_v] = compute_disruption_norm_cons(...
            gamma, v_max, v_min, v_C_0, v_d, t_avg);
        % gamma_x = 0.5; gamma_v = 0.5;
        cost_dis = compute_disruption(gamma_x, gamma_v,...
            x_C_0, v_C_0, x_ego_var, v_ego_var, tf_var);
        % CAVs
        for k=1:numCandidates
            [gamma_x, gamma_v] = compute_disruption_norm_cons(...
                gamma, v_max, v_min, v_0(k,1), v_d, t_avg);
            % gamma_x = 0.5; gamma_v = 0.5;
            cost_dis = cost_dis + compute_disruption(gamma_x, gamma_v,...
                x_0(k,1), v_0(k,1), x_var(k,1), v_var(k,1), tf_var);
        end
        opti.minimize(cost_time + cost_dis);
        
        %% Define Safety Conditions
        for i=1:numCandidates-1
            opti.subject_to(x_var(i,1)-x_var(i+1,1)>=phi*v_var(i+1,1)+delta);
        end 
        %% Define Mergin Constraints
        % Preallocate reachability constraints
        for i=1:numCandidates
            % Merge ahead of i
            opti.subject_to(x_ego_var-x_var(i,1)+(1-B_var(i))*M>=phi*v_var(i,1)+delta);
            % Merge behind of i
            opti.subject_to(x_var(i,1)-x_ego_var+B_var(i)*M>=phi*v_ego_var+delta);
            % Compute reachable set constraint
            [p_u,p_l] = reachable_set_p(u_min, u_max, ...
                x_var(i,1),x_0(i,1),v_var(i,1),v_0(i,1),tf_var);
            opti.subject_to(p_u<=0);
            opti.subject_to(p_l>=0);
            % Speed constraint
            opti.subject_to(v_min<=v_var(i,1)<=v_max);
        end
        %% CAV C safety constraints
        % Vehicle U
        opti.subject_to(x_U_0+v_U_0*tf_var - x_ego_var>=phi*v_ego_var+delta);
        
        % % Vehicle F
        % opti.subject_to(x(1)-x_ego>=phi*v_ego+delta);
        % 
        % % Vehicle B
        % opti.subject_to(x_ego-x(end)>=phi*v(end)+delta);
        
        % Reachability constraint
        [p_u_c,p_l_c] = reachable_set_p(u_min, u_max, ...
                x_ego_var,x_C_0,v_ego_var,v_C_0,tf_var);
        opti.subject_to(p_u_c<=0);
        opti.subject_to(p_l_c>=0);
        opti.subject_to(v_min<=v_ego_var<=v_max);
        % Time Constraints
        opti.subject_to(0<=tf_var<=T_max)
        % Position Constraints (only go forward)
        opti.subject_to(x_var>=x_0)
        opti.subject_to(x_ego_var>=x_C_0)

        % Define optimizer settings
        solver_options = struct('print_time', 1, 'discrete',discrete, 'verbose_init', 1);

        opti.solver('bonmin',solver_options);
        try
            sol = opti.solve();
            tf = sol.value(tf_var);
            x_e_f = sol.value(x_ego_var);
            v_e_f = sol.value(v_ego_var);
            x_f = sol.value(x_var);
            v_f = sol.value(v_var);
            B = sol.value(B_var);
            i_m = find(B)-1;
        catch err
            warning(err.identifier,"%s", err.message)
            tf = 0;
            x_e_f = x_C_0;
            v_e_f = v_C_0;
            x_f = x_0;
            v_f = v_0;
            B = zeros(size(x_0));
            i_m = 0;
        end
        opti.delete
     
end

function [gamma_x, gamma_v] = compute_disruption_norm_cons...
    (gamma, v_max, v_min, v_0, v_d, t_avg)
    gamma_x = gamma/(max(v_max-v_0,v_min-v_0)*t_avg)^2;
    gamma_v = (1-gamma)*max(v_max-v_d, v_min-v_d)^2;
end
function disruption = compute_disruption...
    (gamma_x, gamma_v, x_0, v_0, x_i, v_i, t)
    dis_x = (x_i-(x_0+v_0*t))^2;
    dis_v = (v_i-v_0)^2;
    disruption = gamma_x*dis_x + gamma_v*dis_v;
end

function [p_u_i,p_l_i] = reachable_set_p ...
    (u_min, u_max,x_i,x_i_0,v_i,v_i_0,t)
        nu = (u_max+u_min)/2;
        mu = (u_max-u_min)/2;
        p_u_i = -0.5*t^2 + 0.25*((v_i-v_i_0-nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/nu;
        p_l_i = 0.5*t^2 - 0.25*((-v_i+v_i_0+nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/mu;
end

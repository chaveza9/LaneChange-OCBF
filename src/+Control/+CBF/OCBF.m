classdef OCBF < matlab.System
    %MPC_CBF Summary of this class goes here
    %   Detailed explanation goes her
    properties
        accelMax (1,1) double {mustBeReal, mustBeFinite} = 3.3;    % Vehicle i max acceleration
        accelMin (1,1) double {mustBeReal, mustBeFinite} = -7;   % Vehicle i min acceleration
        velMin (1,1) double {mustBeReal, mustBeFinite} = 13;    % Road min velocity
        velMax (1,1) double {mustBeReal, mustBeFinite} = 33 ;   % Road max velocity 
        tau = 1.2;
        delta_min = 10;
        % Tunning Parameters
        mu_clf = 5;
    end

    properties(SetAccess = 'protected', GetAccess = 'protected')
        % Numerical Solution
        prob = [];
        u_var=[]; 
        z_var=[]; 
        x_p=[]; 
        t_f_p=[]; 
        v_des_p=[]; 
        x_des_p=[];
    end
    
    methods
        function self = OCBF(varargin)
            % OCBF constructor
            % Initializes the construction of the CBF problem in the opti
            % stack
            % Support name-value pair arguments when constructing object
            setProperties(self,nargin,varargin{:})
            % Define QP 
            [self.prob, self.u_var, self.z_var,...
                self.x_p, self.t_f_p, self.v_des_p, self.x_des_p] = ...
                self.define_cbf_qp(); 
        end
        function [status, u] = step(self,...
                collaborative, x_k, x_front_k, x_adj_k, u_ref, v_des, ...
                phi, t_f, x_f)
            status= false;
            if collaborative
                % Solve problem with borth obstacles
                [status, u] = self.solve_fxtm_cbf_2(self.prob.copy(),  ...
                    self.u_var, self.z_var, self.x_p, self.t_f_p, self.v_des_p,...
                    self.x_des_p, x_k, x_front_k, u_ref, t_f, v_des, x_f, x_adj_k, phi);
            end
            if ~status
                    [status, u] = self.solve_fxtm_cbf_1(self.prob.copy(),  ...
                    self.u_var, self.z_var, self.x_p, self.t_f_p, self.v_des_p,...
                    x_k, x_front_k, u_ref, t_f, v_des, x_f);
            end
            if ~status
                error('Solution is Unfeasible')
            end
        end
        
        
    end
    methods(Access=protected)
        function [opti, u_var, z_var, x_p, t_f_p, v_des_p, x_des_p] = define_cbf_qp(self)
            opti = casadi.Opti(); % Optimization problem
        
            %% Setup optimization variables
            % Optimization Variables
            u_var = opti.variable(self.n_controls,1); % control variables [u_ego, u_obst]
            % State Variables
            x_p = opti.parameter(self.n_states*3, 1); % state variables [x_ego, v_ego, x_obst, v_obst]
            % Terminal Time Parameters
            t_f_p =  opti.parameter();
            % Desired speed
            v_des_p =  opti.parameter();
            x_des_p = opti.parameter();
            %% CBF-CLF Parameters
            % Num constraints
            n_cbf = 2; %speed constraints
            n_clf = 2; % Desired Speed, Desired Terminal Position
            %n_total = n_cbf + n_clf;
            % Compute Fixed time guarantees rates
            gamma_1 = 1 + 1/self.mu_clf;
            gamma_2 = 1 - 1/self.mu_clf;
            %% Define Relaxation variables
            slack_clf = opti.variable(n_clf,1); % control variables 
            slack_cbf = opti.variable(n_cbf,1); % control variables 
            % ----------  Compute conditions CLF ----------
            % Define Lyapunov Function
            h_speed = @(x) (x(2) - v_des_p)^2;
            h_pos = @(x) (x(1) - x_des_p)^2;
            % Concatenate functions
            h_goal = {h_pos, h_speed}; 
            % Define qp matrix
            U = [u_var;zeros(2,1)];
            for i =1:length(V)
                % Define lyapunov function
                h_g_i = h_goal{i};
                % Define alpha function
                alpha = mu*pi/(2*t_f_p);
                % Compute clf constraints
                [Lgh_g, Lfh_g] = compute_lie_derivative_1st_order(x_p, h_g_i);
                opti.subject_to(Lgh_g(x_p)*U - h_g_i(x_p)* slack_clf(i) <= ...
                    - Lfh_g(x_p) - alpha*max(0,h_g_i(x_p))^gamma_1 -...
                    alpha*max(0,h_g_i(x_p))^gamma_2);                
            end
    
            % ----------  Compute conditions CBF ---------- 
            % define barrierfunctions
            b_v_min = @(x) -(x(2)-self.velMin);
            b_v_max = @(x) -(self.velMax - x(2));
            % b_dist = @(x) self.Tau*x(2) - (x(3)-x(1));
            h_safe = {b_v_min,b_v_max};            
            for i =1:length(h_safe)
                % Define barrier function
                h_s_i = h_safe{i};
                % Compute CBF constraints
                [Lgh_s, Lfh_s] = compute_lie_derivative_1st_order(x_p, h_s_i);
                opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U <= -slack_cbf(i)*h_s(x_p))
            end
            
            % Add Actuation Limits
            opti.subject_to(u_var>=-self.accelMin)
            opti.subject_to(u_var<= self.accelMax)
            z_var = [u, slack_clf, slack_cbf]';
            opti.delete()
        end
        % CBF with 1 obstacle constraint
        function [status, u] = solve_fxtm_cbf_1(self, opti,  ...
                u_var, z_var, x_p, t_f_p, v_des_p, x_des_p,...
                x_ego, x_front, u_ref, t_f, v_des, x_des)
            % Propagate parametric values
            opti.set_value(x_p, [x_ego; x_front; zeros(2,1)])
            opti.set_value(t_f_p, t_f)
            opti.set_value(v_des_p, v_des)
            opti.set_value(x_des_p, x_des)
            % ----------  Compute conditions CBF ---------- 
            % Define CBF obstacle conditions
            n_cbf = 1;
            % Create new cbf condition
            slack_cbf = opti.variable(n_cbf, 1);
            % Define safety functions
            b_dist_ego_front = @(x) self.tau*x(2) - (x(3)-x(1));
            h_safe = {b_dist_ego_front}; 
            U = [u_var;zeros(2,1)];
            for i =1:length(h_safe)
                % Define barrier function
                h_s_i = h_safe{i};
                % Compute CBF constraints
                [Lgh_s, Lfh_s] = compute_lie_derivative_1st_order(x_p, h_s_i);
                opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U <= -slack_cbf(i)*h_s(x_p))
            end
            % ----------  Compute  qp objective ---------- 
            % Append new variables
            z = [z_var; slack_cbf];
            z(1:self.n_controls) = z(1:self.n_controls) - u_ref;
            % Create quadratic cost
            H_u = 2*eye(self.n_controls);
            H_delta_clf = 2 * eye(2);
            H_delta_cbf = 200 * eye(3);
            H = blkdiag(H_u, H_delta_clf, H_delta_cbf);
            % Linear Cost
            F = [zeros(1, self.n_controls), 2*ones(1,2), zeros(1,3)];
            % Define Objective
            objective = 0.5*z'*H*z+F*z;
            opti.minimize(objective)
            % ----------  Create solver and solve! ---------- 
            opts = self.define_solver_options;
            opti.solver('ipopt',opts)
            try
                solution = opti.solve();
            catch
                status = false;
                u = NaN;
                warning('infeasible problem detected')
                return
            end
            % Populate return values
            u = solution.value(u_var);  
            opti.delete()
        end

         % CBF with 2 obstacle constraint (mergin constraint
        function [status, u] = solve_fxtm_cbf_2(self, opti,  ...
                u_var, z_var, x_p, t_f_p, v_des_p, x_des_p,...
                x_ego, x_front, u_ref, t_f, v_des, x_des, x_adj_front, phi)
            % Propagate parametric values
            opti.set_value(x_p, [x_ego; x_front; x_adj_front])
            opti.set_value(t_f_p, t_f)
            opti.set_value(v_des_p, v_des)
            opti.set_value(x_des_p, x_des)
            % ----------  Compute conditions CBF ---------- 
            % Define CBF obstacle conditions
            n_cbf = 2;
            % Create new cbf condition
            slack_cbf = opti.variable(n_cbf, 1);
            % Define safety functions
            b_dist_ego_front = @(x) self.tau*x(2) - (x(3)-x(1));
            b_dist_ego_adj = @(x) phi*x(2) - (x(5)-x(1));
            h_safe = {b_dist_ego_front, b_dist_ego_adj}; 
            U = [u_var;zeros(2,1)];
            for i =1:length(h_safe)
                % Define barrier function
                h_s_i = h_safe{i};
                % Compute CBF constraints
                [Lgh_s, Lfh_s] = compute_lie_derivative_1st_order(x_p, h_s_i);
                opti.subject_to(Lfh_s(x_p)+Lgh_s(x_p)*U <= -slack_cbf(i)*h_s(x_p))
            end
            % ----------  Compute  qp objective ---------- 
            % Append new variables
            z = [z_var; slack_cbf];
            z(1:self.n_controls) = z(1:self.n_controls) - u_ref;
            % Create quadratic cost
            H_u = 2*eye(self.n_controls);
            H_delta_clf = 2 * eye(2);
            H_delta_cbf = 200 * eye(4);
            H = blkdiag(H_u, H_delta_clf, H_delta_cbf);
            % Linear Cost
            F = [zeros(1, self.n_controls), 2*ones(1,2), zeros(1,3)];
            % Define Objective
            objective = 0.5*z'*H*z+F*z;
            opti.minimize(objective)
            % ----------  Create solver and solve! ---------- 
            opts = self.define_solver_options;
            opti.solver('ipopt',opts)
            try
                solution = opti.solve();
            catch
                status = false;
                u = NaN;
                warning('infeasible problem detected')
                return
            end
            % Populate return values
            u = solution.value(u_var);   
        end

        %% Helper Variables
        function [Lgb, Lfb] = compute_lie_derivative_1st_order(self, ...
                barrier_fun)
        
            %% Define CBF functions
            n_states = self.n_states*3;
            x_s = casadi.MX.sym('x',n_states);
            %% Compute cbf constraints
            alpha_0 = barrier_fun(x_s);
            b = casadi.Function('alpha0',{x_s},...
                      {alpha_0});
            % Compute constraint parameters 
            db_dx = b.jacobian_old(0,0);
            Lfb = casadi.Function('Lfb',{x_s},...
                      {db_dx(x_s)*self.f(x_s)});
            Lgb = casadi.Function('LgLfb',{x_s},...
                      {db_dx(x_s)*self.g(x_s)});
        end
    end

    methods(Access=private, Static)
        function [opts] = define_solver_options()
            % Define solver options
            opts.jit = true;
            opts.compiler = 'shell';
            opts.jit_options.compiler = 'gcc';
            opts.jit_options.flags = {'-O0'};
            opts.qpsol = 'nlpsol';
            opts.qpsol_options.nlpsol = 'ipopt';
            opts.qpsol_options.nlpsol_options.ipopt.tol = 1e-7;
            opts.qpsol_options.nlpsol_options.ipopt.tiny_step_tol = 1e-20;
            opts.qpsol_options.nlpsol_options.ipopt.fixed_variable_treatment = 'make_constraint';
            opts.qpsol_options.nlpsol_options.ipopt.hessian_constant = 'yes';
            opts.qpsol_options.nlpsol_options.ipopt.jac_c_constant = 'yes';
            opts.qpsol_options.nlpsol_options.ipopt.jac_d_constant = 'yes';
            opts.qpsol_options.nlpsol_options.ipopt.accept_every_trial_step = 'yes';
            opts.qpsol_options.nlpsol_options.ipopt.mu_init = 1e-3;
            
            opts.qpsol_options.nlpsol_options.ipopt.print_level = 0;
            %opts.qpsol_options.nlpsol_options.print_time = false;
            opts.qpsol_options.nlpsol_options.ipopt.linear_solver = 'ma27';
            opts.qpsol_options.print_time = true;
        end

         %% Define Control Affine Dynamics
        function x_dot = f(x)
            % [x_1 v_1 x_2 v_2]
            x_dot = [x(2)  
                    0 
                    x(4) 
                    0
                    x(6) 
                    0];
        end
        function  val = g(~)
             val = [0     0     0
                    1     0     0
                    0     0     0
                    0     1     0
                    0     0     0
                    0     0     1];
        end
    end
    
end


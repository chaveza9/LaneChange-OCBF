classdef OCBF < matlab.System
    %MPC_CBF Summary of this class goes here
    %   Detailed explanation goes her
    properties
        % Physical Constraints
        accelMax (1,1) double {mustBeReal, mustBeFinite} = 3.3;    % Vehicle i max acceleration
        accelMin (1,1) double {mustBeReal, mustBeFinite} = -7;   % Vehicle i min acceleration
        omegaMin (1,1) double {mustBeReal, mustBeFinite} = -0.6109;    % Vehicle i min angular rate
        omegaMax (1,1) double {mustBeReal, mustBeFinite} = 0.6109;     % Vehicle i max angular rate
        thetaMin (1,1) double {mustBeReal, mustBeFinite} = -0.1745 % Vehicle i min steering
        thetaMax (1,1) double {mustBeReal, mustBeFinite} = 0.1745;     % Vehicle i max steering
        velMin (1,1) double {mustBeReal, mustBeFinite} = 13;    % Road min velocity
        velMax (1,1) double {mustBeReal, mustBeFinite} = 33 ;   % Road max velocity 
        % Safety Parametersd
        tau = 1.2;
        delta_dist = 10;
        % Simulation Time
        dt = 0.1;
        %Noise Levels 
        w_x (1,1) double {mustBeReal, mustBeFinite} = 0.15; % x Position noise [m]
        w_y (1,1) double {mustBeReal, mustBeFinite} = 0.15; % y Position noise [m]
        w_v (1,1) double {mustBeReal, mustBeFinite} = 0.2; % speed noise [m/s]
        w_theta (1,1) double {mustBeReal, mustBeFinite} = 0.05; % heading noise [rad]
        % Vehicle Parameters
        wheelBase (1,1) double {mustBeReal, mustBeFinite} = 5 % Vehicle wheel base Lw [m]
        % Tunning Parameters
        mu_clf = 5;
        % Control variables
        n_controls = 2; %[acc, omega]
        n_states = 4; %[pos, vel] [m, m/s]
        uncooperative = 0;
        % Method Parameters
        split = 0
        method (1,:) char {mustBeMember(method,{'fxtm','cbf','ocbf'})} = 'fxtm'
    end

    methods
        function self = OCBF(varargin)
            % OCBF constructor
            % Initializes the construction of the CBF problem in the opti
            % stack
            % Support name-value pair arguments when constructing object
            setProperties(self,nargin,varargin{:})
        end
        
    end
    methods(Access=protected)
        function [status, u] = stepImpl(self,...
                collaborative, x_ego_k, x_front_k, x_adj_k, u_ref, v_des, ...
                phi, t_f, x_des, y_des, flag)
            arguments
                self
                collaborative
                x_ego_k
                x_front_k
                x_adj_k
                u_ref
                v_des
                phi
                t_f
                x_des
                y_des
                flag = false;
            end
            status= false;
            if self.split %|| self.uncooperative
                % Compute CLF
                [~, u_ref_clf] = self.solve_fxtm_clf(...
                    x_ego_k, x_front_k, u_ref, t_f, v_des, x_des, y_des, flag);
                if collaborative
                    % Solve problem with both obstacles
                    [status, u] = self.solve_cbf_2(x_ego_k, x_front_k, ...
                        u_ref_clf, x_adj_k, phi);
                end
                if ~status
                     [status, u] = self.solve_cbf_1(...
                            x_ego_k, x_front_k, u_ref_clf);
                end
            else
                % Compute Safety Constraints
                if collaborative
                    % Solve problem with both obstacles
                    [status, u] = self.solve_fxtm_clbf_2(...
                        x_ego_k, x_front_k, u_ref, t_f, v_des, x_des,...
                        x_adj_k, phi, y_des, flag);
                end
                if ~status
                    [status, u] = self.solve_fxtm_clbf_1(x_ego_k, x_front_k,...
                        u_ref, t_f, v_des, x_des, y_des, flag);
                end
                if ~status
                    [status, u] = self.solve_cbf_1(...
                            x_ego_k, x_front_k, u);
                end
            end
            % Check if solution is valid
            if ~status
                warning('Solution is Unfeasible')
                u = [-7,0];
            end
        end
        
        % Implemented outside
        % CLF
        [status, u] = solve_fxtm_clf(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des, y_des, flag)
        % CBF
        [status, u] = solve_cbf_1(self, x_ego, x_front, u_ref)
        [status, u] = solve_cbf_2(self, x_ego, x_front, u_ref,...
            x_adj_front, phi)
        % CLBF
        [status, u] = solve_fxtm_clbf_2(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des,...
            x_adj_front, phi, y_des, flag)
        [status, u] = solve_fxtm_clbf_1(self, ...
            x_ego, x_front, u_ref, t_f, v_des, ...
            x_des, y_des, flag)
        %% Helper Variables
        function [Lgb, Lfb, Lwb] = compute_lie_derivative_1st_order(self, ...
                barrier_fun)
        
            %% Define system states
            x_s = casadi.MX.sym('x', self.n_states+4);
            %% Compute cbf constraints
            alpha_0 = barrier_fun(x_s);
            b = casadi.Function('alpha0',{x_s},...
                      {alpha_0});
            % Compute constraint parameters 
            db_dx = b.jacobian_old(0,0);
            Lfb = casadi.Function('Lfb',{x_s},...
                      {db_dx(x_s)*self.f(x_s)});
            Lgb = casadi.Function('LgLb',{x_s},...
                      {db_dx(x_s)*self.g(x_s)});
            Lwb = casadi.Function('LwLb',{x_s},...
                      {db_dx(x_s)*self.W(x_s)});
        end

                 %% Define Control Affine Dynamics
        function x_dot = f(~,x)
            % [x_1 v_1 x_2 v_2]
            x1_dot =[x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
            x2_dot = [x(6), 0]';
            x3_dot = [x(8), 0]';
            x_dot = [x1_dot;x2_dot;x3_dot];
        end
        function  G = g(self, x)
            b1 = [0 , -x(3)*sin(x(4));
                  0  ,  x(3)*cos(x(4));
                  1  ,  0;
                  0  ,  x(3)/self.wheelBase];
            b2 = [0;1];
            b3 = [0;1];
            
            G = blkdiag(b1,b2,b3);
        end
        function H = W(self, x)
            z = length(x)-self.n_states;
            H = blkdiag(eye(self.n_states),eye(z)); 
        end
        function [b,H] = noise_lims(self)
            % Hw<=b
            vals = [1;-1];
            b = [vals*self.w_x % ego x
                 vals*self.w_y % ego y
                 vals*self.w_v % ego v
                 vals*self.w_theta % ego theta
                 vals*self.w_x % front x
                 vals*self.w_v % front v
                 vals*self.w_x % adj x
                 vals*self.w_v % adj v
                 ];
            H =  blkdiag(vals,vals,vals,vals,vals,vals,vals,vals);
        end

    end

    methods(Access=private, Static)
        function [opts] = define_solver_options()
            % Define solver options
            if ismac
                opts.jit = true;
                opts.compiler = 'shell';
                opts.jit_options.compiler = 'gcc';
                opts.jit_options.flags = {'-O0'};
            end
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
            opts.qpsol_options.nlpsol_options.print_time = false;
            % opts.qpsol_options.nlpsol_options.ipopt.linear_solver = 'ma27';
            opts.qpsol_options.print_time = false;
        end

    end
    
end


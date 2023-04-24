classdef OCBF < matlab.System
    %MPC_CBF Summary of this class goes here
    %   Detailed explanation goes her
    properties
        accelMax (1,1) double {mustBeReal, mustBeFinite} = 3.3;     % Vehicle i max acceleration
        accelMin (1,1) double {mustBeReal, mustBeFinite} = -7;      % Vehicle i min acceleration
        omegaMin (1,1) double {mustBeReal, mustBeFinite} = -0.3;    % Vehicle i min angular rate
        omegaMax (1,1) double {mustBeReal, mustBeFinite} = 0.3;     % Vehicle i max angular rate
        velMin (1,1) double {mustBeReal, mustBeFinite} = 13;    % Road min velocity
        velMax (1,1) double {mustBeReal, mustBeFinite} = 33 ;   % Road max velocity 
        tau = 1.2;
        dt = 0.1;
        delta_dist = 10;
        % Tunning Parameters
        mu_clf = 5;
        % Control variables
        n_controls = 2; %[acc]
        n_states = 4; %[x,y,vel, theta] [m,m,m/s,rad]
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
                phi, t_f, x_des, theta_des)
            status= false;
            if collaborative
                % Solve problem with borth obstacles
                [status, u] = self.solve_fxtm_cbf_2(x_ego_k, x_front_k, ...
                    u_ref, t_f, v_des, x_des, x_adj_k, phi, theta_des);
            end
            if ~status
                [status, u] = self.solve_fxtm_cbf_1(...
                    x_ego_k, x_front_k, u_ref, t_f, v_des, x_des, theta_des);
            end
            if ~status
                error('Solution is Unfeasible')
            end
        end
        
        % Implemented outside
        [status, u] = solve_fxtm_cbf_1(self, ...
            x_ego, x_front, u_ref, t_f, v_des, x_des, theta_des)
        [status, u] = solve_fxtm_cbf_2(self, ...
             x_ego, x_front, u_ref, t_f, v_des, x_des, x_adj_front,...
             phi, theta_des)
        %% Helper Variables
        function [Lgb, Lfb] = compute_lie_derivative_1st_order(self, ...
                barrier_fun)
        
            %% Define CBF functions
            x_s = casadi.MX.sym('x', self.n_states+4);
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

         %% Define Control Affine Dynamics
        % function x_dot = f(x)
        %     % [x_1 v_1 x_2 v_2]
        %     x_dot = [x(2)  
        %             0 
        %             x(4) 
        %             0
        %             x(6) 
        %             0];
        % end
        % function  val = g(~)
        %      val = [0     0     0
        %             1     0     0
        %             0     0     0
        %             0     1     0
        %             0     0     0
        %             0     0     1];
        % end
        function x_dot = f(x)
            x1_dot = [x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
            x2_dot = [x(6), 0]';
            x3_dot = [x(8), 0]';
            x_dot = [x1_dot;x2_dot;x3_dot];
        end
        
        function b = g(~)
            b1 = [zeros(2,2);
                 eye(2)];
            b2 = [0;1];
            b3 = [0;1];

            b = blkdiag(b1,b2,b3);
        end
    end
    
end


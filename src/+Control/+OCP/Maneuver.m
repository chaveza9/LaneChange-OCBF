 classdef Maneuver < matlab.System
    % Defines a object that computes a longitudinal and lateral maneuver
    % for a lane-changing maneuver
    properties(SetAccess = 'public', GetAccess = 'public')
        accelMax (1,1) double {mustBeReal, mustBeFinite} = 3.3;     % Vehicle i max acceleration
        accelMin (1,1) double {mustBeReal, mustBeFinite} = -7;      % Vehicle i min acceleration
        velMin (1,1) double {mustBeReal, mustBeFinite} = 13;        % Road min velocity
        velMax (1,1) double {mustBeReal, mustBeFinite} = 33 ;       % Road max velocity 
        % Constraints tolerance
        tol_v_des (1,1) double {mustBeReal, mustBeFinite} = 3 ;   % Delta tolerance desired speed
        tol_x_des (1,1) double {mustBeReal, mustBeFinite} = 2 ;   % Delta tolerance final position
    end
    properties (Access = public)
        % CAV Type
        cavType (1,:) char {mustBeMember(cavType,{'cav1','cav2', 'cavC','none'})} ='none'
        N = 250;
        dt = 0.1;
        tau = 1.2                % Reaction time [s]
        delta_dist = 4.2         % Min Safety distance [m]
        delta_tol = 3            % Tolerance Delta speed 
        alpha_energy = 0.4       % Energy Cost gain
        Verbose = false          % print solution iteration        
    end

    %% Properties
    properties(SetAccess = 'protected', GetAccess = 'public')
        % Optimal Control Trajectory
        u_opt = [];
        x_opt = [];
    end
    properties(SetAccess = 'protected', GetAccess = 'protected')
        % Terminal States
        terminalTime (1,1) {mustBeReal} = NaN;
        terminalPosition (1,1) {mustBeReal} = NaN;
        % Numerical Solution
        problem = [];
        x_var= []; 
        u_var=[]; 
        x_obs_par = [];
        tf_par = [];
        v_des_par = [];
    end
    %% Public access methods
    methods      
        function self = Maneuver(varargin)
            % initialize Vehicle superclass
            setProperties(self,varargin{:});
        end

        function [status, u_hist, x_hist, t_hist] = ...
                compute_ocp(self, currState, tf, xf, v_des, x_obst_front, verbose)
            arguments
                self
                currState (2,1) double {mustBeReal, mustBeFinite}
                tf (1,1) double {mustBeReal, mustBeFinite}
                xf (1,1) double {mustBeReal, mustBeFinite}
                v_des (1,1) double {mustBeReal, mustBeFinite}
                x_obst_front (2,1) double {mustBeReal, mustBeFinite} = [NaN, NaN]
                verbose = false;
            end
            % Deal with no obstacle in front
            if any(isnan(x_obst_front))
                x_obst_front = currState;
                x_obst_front(1) = currState(1) + 1000; 
            end
            
            % Compute OCP trajectory for CAV
            [status, u_hist, x_hist] = self.solve_ocp(...
                self.problem.copy, self.x_var, self.u_var, ...
                self.x_obs_par, self.tf_par, self.v_des_par,...
                currState, x_obst_front, xf, tf, v_des, ...
                self.tol_v_des, self.tol_x_des, verbose);
            % Define time history
            t_hist = linspace(0, tf, size(x_hist,2));
            self.u_opt = u_hist;
            self.x_opt = x_hist; 
            self.terminalTime = tf;
            self.terminalPosition = xf;

        end
        
        function [posX, speed, accel] = extract_cntrl_input(self, currState, time)
            % check if judge exists
            if isempty(self.u_opt)
                error('Maneuver needs to be started first')
            end
            optCntrl = self.u_opt;
            % Compute current vehicle state
       
            x_0 = currState(1);     % Vehicle initial position
            v_0 = currState(2);     % Vehicle initial velocity
            % Extract current time
            t0 = 0;
            tf = self.terminalTime+t0;
            self.N = length(optCntrl);
            % Create time vector
            time_vec = linspace(t0,tf,self.N);
            % Compute Control strategy at current time step
            if time <= tf
                accel = interpn(time_vec,optCntrl,time);
            else
                accel = optCntrl(end);
            end
            %% Declare model variables
            % Model equations
            xdot = @(t,x,u)[x(2);u];
            tk_1 = time;
            tk = tk_1+self.dt;
            tspan = [tk_1 ,tk];
            [~,y] = ode45(@(t,y)xdot(t,y,accel),tspan,[x_0; v_0]);
            % Extract X(tk)
            posX = y(end,1);
            speed = y(end,2);
        end    
    end
    
    methods(Access=protected, Static)
        function x_next = runge_kutta4(f, x, u, dt)
            % Runge-Kutta 4 integration
            k1 = f(x,         u);
            k2 = f(x+dt/2*k1, u);
            k3 = f(x+dt/2*k2, u);
            k4 = f(x+dt*k3,   u);
            x_next = x + dt/6*(k1+2*k2+2*k3+k4);
        end
        function [status, u, x] = solve_ocp(...
                opti, x_var, u_var, X_obst_var, tf_var, v_des_var,...
                curr_state, x_obs_0, xf, tf, v_des, tol_speed,...
                tol_pos, verbose)
            % Define Initial Condition
            opti.subject_to(x_var(:,1) == curr_state)
            % Define Terminal Condition
            pos = x_var(1,:);
            speed = x_var(2,:);
            opti.subject_to((pos(end) - xf)^2 <= tol_pos^2)
            opti.subject_to((speed(end)- v_des_var)^2 <= tol_speed^2)
            % Populate Parameters
            opti.set_value(X_obst_var, x_obs_0)
            opti.set_value(tf_var, tf)
            opti.set_value(v_des_var, v_des)
            opti.set_initial(u_var, 3.3); 
            % Define optimizer settings
            if verbose
                tree_level = 3;
            else
                tree_level = 0;
            end
            opti.solver('ipopt',struct('print_time',verbose,'ipopt',...
            struct('max_iter',10000,'acceptable_tol',1e-8, ...
            'print_level',tree_level,'acceptable_obj_change_tol',1e-6))); % set numerical backend
            
            % Create solver and solve!
            try
                % solution = opti.solve();
                solution = opti.solve_limited();
            catch
                status = false;
                x = NaN;
                u = NaN;
                warning('infeasible problem detected')
                return
            end
            % Populate return values
            status = solution.stats.success;
            x = solution.value(x_var);
            u = solution.value(u_var);   
            
        end
    end
    methods(Abstract, Access = 'public')
        % Defines a parametric model of optimal control problem
        [opti, X, U, X_obs_0, tf, v_des] = define_problem(self)
    end
end


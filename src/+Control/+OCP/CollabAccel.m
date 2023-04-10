classdef CollabAccel < Control.OCP.Maneuver
    % Acceleration planning maneuver
    %   Creates an object containing a numerical solution of optimal
    %   control problem for an acceleration maneuver. It inherits all the
    %   methods from "Maneuver" Class
    
    
    %% Public access methods
    methods(Access = 'public')
        function self = CollabAccel(varargin)
            % initialize Vehicle superclass
            self = self@Control.OCP.Maneuver();
            setProperties(self,nargin,varargin{:});
            % Define Parametric Model of the problem
            [self.problem, self.x_var, self.u_var, self.x_obs_par, ...
                self.tf_par, self.v_des_par] = self.define_problem();
        end

        
        function [opti, X, U, X_obs_0, tf_p, v_des] = define_problem(self)
            %% Import libraries
            import casadi.*
            %% Define Optimization Variables
            % Call opti from casadi
            opti = casadi.Opti;
            % Define parameter
            v_des = opti.parameter();
            %% Extract Constraints
            u_max = self.accelMax;    % Vehicle i max acceleration
            u_min = self.accelMin;   % Vehicle i min acceleration
            v_min = self.velMin;   % Vehicle i min velocity
            v_max = self.velMax;   % Vehicle i max velocity
            %% Define Cost gains for convex combination
            gamma_energy = (self.alpha_energy)/max([u_max, u_min].^2);
            gamma_speed = (1-self.alpha_energy)/max([v_max-v_des, v_min-v_des].^2);
           
            % Define decision variables
            X = opti.variable(2,self.N+1); % state trajectory
            pos   = X(1,:);
            speed = X(2,:);
            U = opti.variable(1,self.N);   % control trajectory (throttle)
            % Front Obstacle Vehicle
            X_obs_0 = opti.parameter(2,1); % Obstacle Vehicle Initial Conditions
            % Terminal Time Parametric Variable
            tf_p = opti.parameter(); % Vehicle terminal time
            % Define dynamic constraints
            c_u = @(u, x) u^2; % Cost Acceleration
            c_v = @(u, x) u;
            f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
            % Define objective funciton integrator
            % Integrate using RK4
            dt = tf_p/self.N;
            % cost = beta*(speed(end)-v_des)^2;
            cost = 0;
            for k=1:self.N % loop over control intervals
                % Forward integration
                X_next = self.runge_kutta4(f, X(:,k), U(:,k), dt);
                cost = cost + 0.5*gamma_energy*self.runge_kutta4(c_u, U(:,k), X(:,k), dt);
                cost = cost + gamma_speed*self.runge_kutta4(c_v,(speed(:,k)-v_des)^2,X(:,k),dt);
                % Impose multi-shoot constraint
                opti.subject_to(X(:,k+1)==X_next); % close the gaps
                % Impose Front Vehicle Safety Constraint
                x_obs_k = X_obs_0(1)+X_obs_0(2)*dt*k;
                opti.subject_to(x_obs_k - pos(k+1)>=...
                    speed(k+1)*self.tau+self.delta_dist); % safety constraint
            end
            % ----- Objective function ---------
            opti.minimize(cost);
            % --------- Define path constraints ---------
            opti.subject_to(v_min<=speed<=v_max);     %#ok<CHAIN> % track speed limit
            opti.subject_to(u_min<=U<=u_max);     %#ok<CHAIN> % control is limited
            % --------- Define Boundary conditions ---------
            % opti.subject_to(pos(1)== X_0);   % start at position 0 ...
            % opti.subject_to(speed(1)== v_0); % initial speed
            % % --------- Terminal Constraints ----------------------
            % opti.subject_to(pos(end) == pos_f)
            % Warm Start solver
            % opti.set_initial(speed, 20);
            opti.set_initial(U, 2); 
        end
    end
end


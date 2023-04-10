classdef VehicleDynamics < matlab.System
    % Vehicle Class
    %   Computes the dynamics of vehicle
    properties (SetAccess = 'protected', GetAccess = 'public')
        x_0 (4,1) double {mustBeReal, mustBeFinite}  %[PosX PosY, Speed, theta] [m m m/s rad]
        %current state %[PosX PosY, Speed, theta] [m m m/s rad]
        x_k  (4,1) double {mustBeReal, mustBeFinite} 
        % Time step
        dt (1,1) double {mustBeReal, mustBeFinite} = 0.01
        % Current time
        t_k(1,1) double {mustBeReal, mustBeFinite} = 0
    end
   
    % Constant properties 
    properties(SetAccess = 'public', GetAccess = 'public')     
        % Bicycle Model
        vehLength (1,1) double {mustBeReal, mustBeFinite} = 4.3
        vehWidth (1,1) double {mustBeReal, mustBeFinite} = 1.7
        vehWheelBase (1,1) double {mustBeReal, mustBeFinite} = 5
        speedRange (1,2) double {mustBeReal, mustBeFinite} = [-20 40]
        
    end
    % provate properties
    properties(SetAccess = 'protected', GetAccess = 'public')
        % State history
        x_hist (:,5) double {mustBeReal, mustBeFinite} = []; %[PosX PosY, Speed, theta] [sec m m m/s m/s, rad, rad]
        u_hist (:,2) double {mustBeReal, mustBeFinite} = []; %[acc, omega] [sec m/s^2 rad/s]
        t_hist (:,1) double {mustBeReal, mustBeFinite} = []; %[time] [sec]
    end
    
    methods
        % Constructor
        function self = VehicleDynamics(x_0, varargin)
            % Initial condition vehicle
            self.x_0 = x_0;
            % Append initial condition to state history
            self.x_k = x_0;
            self.x_hist = cat(2, self.x_hist, self.x_k);
            self.t_hist = cat(2, self.t_hist, self.t_k);
            % Setup variable public properties
            if nargin > 1
                setProperties(self,nargin-1,varargin{:});
            end
        end 
    end
    methods (Access = public)
        function x_curr = integrate_forward(self, varargin)
            % Computes the step of a model using the predefined kinematic model
            % at dt. Uses ODE45 with the last time step
           if nargin == 1
               u_k = [0, 0]';
           else
               u_k = varargin{:};
           end
            % Propagate Time
            t_k_1 = self.t_k;
            self.t_k = self.t_k+self.dt;
            % Applies control input to the specified model dynamics
            % Integrate forward
            tspan = [t_k_1 ,self.t_k];
            [~,y] = ode45(@(t,y)derivative(self.dynamics,y,u_k),...
                tspan,self.x_k);
            % Extract X(tk)
            self.x_k = y(end, :);
            % Update current state and current time
            self.add_state_history(self.x_k, u_k, self.t_k)
            x_curr = self.x_k;
        end

        function fig = generate_state_history_plot(self, name)
            % Generates state history plot with title name            
            fig = figure('Name',name,'Tag','ScenarioStateHistory');
            fig.Position = [50 70 1500 900];
            time = self.t_hist; %[sec]
            % Speed profile
            % Gen speed history
            speedHist = self.x_hist(3,:);
            
            xHist = self.x_hist(1,:);
            subplot(2,1,1)
                plot(time,speedHist)
                hold on
                grid minor
                title(name)
                ylabel('Speed [m/sec]')
                ylim([min(speedHist)-4, max(speedHist)+4])
            % Position
            subplot(2,1,2)
                plot(time,xHist) % Posx
                hold on
                grid minor
                xlabel('Time [sec]')
                ylabel('X-Position [m]')
                ylim([min(self.x_hist(:,2))-2,max(self.x_hist(:,2))+2])
            
        end
       
    end
    
    methods (Access = private)
        
        function add_state_history(self, x_k, u_k, t)
            % Decompose states
            self.x_hist = cat(2, self.x_hist, x_k);
            self.t_hist = cat(2, self.t_hist, t);
            self.u_hist = cat(2, self.u_hist, u_k);
        end
    end

    methods (Static)
        function x_dot = dynamics(~, x, u)
            % compute derivatives
            x_dot = f(x)+g(x)*u;
        end
        
        function x_dot = f(x)
            x_dot = [x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
        end
        
        function val = g(~)
            val = [zeros(2,2);
                 eye(2)];
        end
    end
    


    
end



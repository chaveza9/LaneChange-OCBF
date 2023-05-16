classdef IntelligentVehicle < handle
    %INTELLIGENTVEHICLE Summary of this class goes here
    %   Detailed explanation goes here

    %TODO Propagate vehicle dynamics through initialization on Intelligent
    %Vehicle
    properties (SetAccess = protected, GetAccess = public)
        % Vehicle Metadata
        % Vehicle name
        VehicleID (1,:) char {mustBeText}
        % Vehicle type (is manned or non manned)
        VehicleType (1,:) char {mustBeMember(VehicleType,{'NonControlled','CAV'})} = 'NonControlled'
        % Vehicle Initial conditions
        InitialConditions =  struct('Position',[],...
                                    'Velocity',[],...
                                    'Heading',[])
        % VehicleConstraints
        AccelMax
        AccelMin
        VelMax
        VelMin

        %Noise Levels 
        w_x (1,1) double {mustBeReal, mustBeFinite} = 0.2*0; % x Position noise [m]
        w_y (1,1) double {mustBeReal, mustBeFinite} = 0.0; % y Position noise [m]
        w_v (1,1) double {mustBeReal, mustBeFinite} = 0.2*0; % speed noise [m/s]
        w_theta (1,1) double {mustBeReal, mustBeFinite} = 0.00; % heading noise [rad]

        % Rear Safety Distance
        MinSafetyDistance (1,:) double {mustBeNumeric, mustBePositive}= 5;
        ReactionTime (1,:) double {mustBeNumeric, mustBePositive}= 0.9;

        % Simulation Metadata
        % Simulation sample time
        SampleTime double {mustBeNonnegative, mustBeNumeric} = 0.01;
        % Simulation current time
        CurrentTime double {mustBeNonnegative, mustBeNumeric} = 0.0;
        % Stop Time
        StopTime double {mustBeNonnegative, mustBeNumeric} = 100;
        
        % Operational Parameters
        % Vehicle Dynamics
        Dynamics  % Vehicle Dynamics
        % Vehicle driving behaviour
        DrivingBehaviour (1,:) char {mustBeMember(DrivingBehaviour,{'collaborating','selfish' })} = 'selfish'
        % Vehicle states
        CurrentState = struct('Position',[],...
                              'Velocity',[],...
                              'Heading',[])

        % Vehicle roll if CAV once request has been set
        RollType (1,:) char {mustBeMember(RollType,{'none','cav1','cav2','cavC'})} = 'none';
        IsCollaborating logical {mustBeNumericOrLogical} =  false;
        HasAborted logical {mustBeNumericOrLogical} = false;
        StartLatManeuver logical {mustBeNumericOrLogical} = false;
        % Driving Scenario Designer
        Scenario
        Vehicle
        % Leader Related Variables
        Leader = [];
        IsLeader logical {mustBeNumericOrLogical} = true;
        % Has arrived to destination lane
        LaneChangeTime_start = NaN;
        LaneChangeTime_end = NaN;
        % Difference between control inputs
        u_diff_hist = [];
       
    end

    properties (SetAccess = private, GetAccess = public)
        % Control Instances Objects
        ocp_prob = []
        cbf_prob = []
        % Terminal Time for Maneuver
        t_0 = 0;
        t_f = NaN;
        x_f = NaN;
        x_0 = NaN;
        % Desired Speed
        DesSpeed = 33;
        % Desired Y Position 
        DesYPos = 1.8; %[m]
        AdaptiveSafetyCheck = 0;
        % Collaboration vehicles id
        Ego_cav_id = [];
        Front_cav_id = [];
        Rear_cav_id = [];
        cav_env= [];
    end


    methods
        function self = IntelligentVehicle(VehID, VehicleScenario, ...
                InitialConditions, StopTime, constraints, Options)
            %INTELLIGENTVEHICLE Construct an instance of this class
            arguments
                VehID (1,:) char {mustBeText}
                VehicleScenario {mustBeUnderlyingType(VehicleScenario, "drivingScenario")}
                InitialConditions struct
                StopTime double {mustBeNonnegative, mustBeNumeric}
                constraints
                Options.UDecel double {mustBeNumeric} = 0.00;
                Options.SampleTime double {mustBeNonnegative, mustBeNumeric} = 0.01;
                Options.VehicleType (1,:) char {mustBeMember(Options.VehicleType,{'NonControlled','CAV'})} = 'CAV'
                Options.SafetyDistance (1,:) double {mustBeNumeric, mustBePositive}= 10;
                Options.ReactionTime (1,:) double {mustBeNumeric, mustBePositive}= 0.9;
                Options.VehicleClass (1,:) {mustBeInteger} = 1; % 1= car. 2=truck
                Options.VehicleLength (1,:) double {mustBeNumeric, mustBePositive}= 4.7; %[m]
                Options.VehicleRearOverhang (1,:) double {mustBeNumeric, mustBePositive}= 1; %[m]
            end
            % _________Initiate Values ________________
            % Propagate constratints
            self.AccelMax = constraints.u_max;
            self.AccelMin = constraints.u_min;
            self.VelMax = constraints.v_max;
            self.VelMin = constraints.v_min;
            % Initialize required variables
            self.VehicleID = VehID;
            self.Scenario = VehicleScenario;
            % Optional variables
            self.SampleTime = Options.SampleTime;
            self.StopTime = StopTime;
            self.VehicleType = Options.VehicleType;
            self.MinSafetyDistance = Options.SafetyDistance;
            self.ReactionTime = Options.ReactionTime;
            % _________Initialize Vehicle Model ________________
            self.CurrentState = InitialConditions;
            initStates = self.decompose_state(self.CurrentState);
            % Select mesh depending on class ID
            if Options.VehicleClass == 2
                carMesh = driving.scenario.truckMesh;
            else
                carMesh = driving.scenario.carMesh;
            end
            % _________Initialize Vehicle Model ________________
            self.Vehicle = vehicle(self.Scenario, ...
                'ClassID', 1, ...
                'Position', [self.CurrentState.Position' 0], ...
                'Velocity',[self.CurrentState.Velocity 0 0],...
                'Yaw', self.CurrentState.Heading, ...
                'Mesh', driving.scenario.carMesh, ...
                'Name', self.VehicleID,...
                'Mesh',carMesh,...
                'Length',Options.VehicleLength,...
                'RearOverhang',Options.VehicleRearOverhang);
            % make sure scenario and sample time are the same
            self.Scenario.SampleTime = self.SampleTime;
            % link simulation time to scenario
            self.CurrentTime = self.Scenario.SimulationTime;
            % initialize leader related variables
            [self.Leader, self.IsLeader] = self.find_leader();
            % Change color if leader
            if strcmp(self.VehicleType,'CAV')
                self.Vehicle.PlotColor = 'cyan';
            else
                self.Vehicle.PlotColor = 'black';
            end
            % _________Initialize Dynamics_______________
            self.Dynamics = Dynamics.VehicleDynamics(initStates,...
                'dt',self.SampleTime, 't_k', self.CurrentTime,...
                'vehWheelBase',self.Vehicle.Wheelbase,...
                'VelMin', self.VelMin,...
                'w_x', self.w_x,...
                'w_y', self.w_y,...
                'w_v', self.w_v,...
                'w_theta', self.w_theta, ...
                'decel', Options.UDecel);
            % _________Initialize OCP and CBF models_______________
            self.ocp_prob = Control.OCP.CollabAccel(...
                'accelMax', self.AccelMax, ...
                'accelMin', self.AccelMin, ...
                'velMax', self.VelMax, ...
                'velMin', self.VelMin,...
                'tau', self.ReactionTime,...
                'delta_dist', self.MinSafetyDistance,...
                'dt', self.SampleTime);
            self.cbf_prob = Control.CBF.OCBF(...
                'accelMax', self.AccelMax, ...
                'accelMin', self.AccelMin, ...
                'velMax', self.VelMax, ...
                'velMin', self.VelMin,...
                'tau', self.ReactionTime,...
                'delta_dist', self.MinSafetyDistance,...
                'w_x', self.w_x,...
                'w_y', self.w_y,...
                'w_v', self.w_v,...
                'w_theta', self.w_theta,...
                'dt', self.SampleTime);
        end %constructor

        
        function hasDefinedCavRoll = define_cav_roll (...
                self, cavType, tf, xf, v_des, cav_env, option)
            % Defines the maneuver roll of the vehicle. This can be of type
            %  {'Acceleration','Deceleration','Social','Selfish', 
            % 'SelfishRelaxed'})}. For this purpose a desired speed
            % "desSpeed" is defined as a speed to follow. In the background
            % the ocp is solved to compute an ideal reference trajectory
            % -------------------------------------------------------------
            
            % Validate Arguments
            arguments
                self
                cavType {mustBeMember(cavType,{'cav1','cav2','cavC'})}
                tf
                xf
                v_des = 33;  
                cav_env = []
                option.e_collab_id = NaN;
                option.f_collab_id = NaN;
                option.r_collab_id = NaN;
                option.verbose = false;
            end
            % Propagate cav environment
            self.cav_env = cav_env;
            % Sanity Check to make sure that ids have been stablished
            % properly
            e_collab_id = option.e_collab_id;
            f_collab_id = option.f_collab_id;
            r_collab_id = option.r_collab_id;
            if strcmp(cavType, 'cav2') && isnan(e_collab_id)
                error('need to define a proper id for CAV C')
            elseif strcmp(cavType, 'cavC') && (isnan(r_collab_id)||isnan(f_collab_id))
                error('need to define a proper id for CAV C')
            end

            % Create initial conditions vector
            x_0_ego = self.contruct_integrator_states(self.CurrentState);
            % Extract Leader State
            [self.Leader, self.IsLeader] = self.find_leader();
            if self.IsLeader
                x_obst_leader = x_0_ego;
                x_obst_leader(1) = x_obst_leader(1) + 1000;
            else
                posU = self.Leader.Position(1);
                velU = norm(self.Leader.Velocity(1:2));
                x_obst_leader = [posU, velU]';
            end
            % Compute Optimal Control Solution
            hasDefinedCavRoll = self.ocp_prob.compute_ocp(...
                x_0_ego, tf, xf, v_des, x_obst_leader, option.verbose);
            
            % Update vehicle state
            self.DesSpeed = v_des;
            self.RollType = cavType;
            self.t_f = tf;
            self.x_f = xf;
            self.x_0 = x_0_ego;
            self.t_0 = self.CurrentTime;
            if hasDefinedCavRoll
                self.IsCollaborating = hasDefinedCavRoll;
                self.DrivingBehaviour = 'collaborating';
                % Define color type based on cav type
                switch cavType
                    case 'cavC'
                        self.Vehicle.PlotColor = 'red';
                    case 'cav2'
                        self.Vehicle.PlotColor = 'green';
                end
            end
            % Propagate collaboration ids
            self.Ego_cav_id = e_collab_id;
            self.Front_cav_id = f_collab_id;
            self.Rear_cav_id = r_collab_id;
        end % define_cav_roll
        
        function isRunning = step(self)
            % Performs an update step and increases time step by one.
            % Additionally, it updates vehicle position on DSD scenario and
            % behaves according to current behaviour type
            % -------------------------------------------------------------
            % ----------- Check if Sim is still running -------------------
            if self.CurrentTime > self.StopTime
                isRunning = false;
                warning('Simulation has reached defined final time')
                return
            end
            % ----------- Vehicle is not collaborating --------------------
            if strcmp(self.DrivingBehaviour,'selfish') 
                % Apply constant control input
                x_k = self.Dynamics.integrate_forward();
                self.CurrentState = self.construct_state_structure(x_k);
                % update Vehicle on DSD
                self.update_dsd_vehicle_states();
                isRunning = true;
                % Update Time Step
                self.CurrentTime = self.CurrentTime + self.SampleTime;
                return
            end
            % ----------- Vehicle is collaborating ------------------------
            % Create initial conditions vector
            % x_k_ego = self.contruct_integrator_states(self.CurrentState);
            x_k_ego = self.contruct_bicycle_state(self.CurrentState);
            % Extract leader's state (if it exists)
            [self.Leader, self.IsLeader] = self.find_leader();
            if self.IsLeader
                x_k_lead = x_k_ego([1,3]);
                x_k_lead(1) = x_k_lead(1) + 1000;
            else
                posU = self.Leader.Position(1);
                velU = norm(self.Leader.Velocity(1:2));
                x_k_lead = [posU, velU]';
            end
            % Compute u_reference function based on current state
            [~, v_ref, u_ref] = self.ocp_prob.extract_cntrl_input(...
                x_k_ego, self.CurrentTime);
            % Compute remaining time from terminal time
            t_des = max(abs(self.t_f - (self.CurrentTime - self.t_0)), 0.01);
            % t_des = self.t_f;
            x_des = self.x_f;
            y_des = self.DesYPos;
            % u_ref=  0;
            % Compute CBF control input based on reference signal    
            switch self.RollType
                % CAV 1 is only safe with respect to vehicle in front
                case 'cav1'
                    collab = 0;
                    phi = 0;
                    % v_ref = self.DesSpeed+2;
                    x_k_adj = x_k_ego([1,3])*0;
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des, y_des);
                % CAV 1 is safe with respect to Leader vehicle (cav 1 and
                % cav C terminal position
                case 'cav2'
                    collab = 1;
                    % v_ref = self.DesSpeed;
                    % Extract adj vehicle
                    adj_cav = self.find_vehicle_from_id(self.Ego_cav_id);
                    Lm = adj_cav.x_f - adj_cav.x_0(1);
                    Li = self.x_f - self.x_0(1);
                    if norm(x_k_ego(1)-x_des)>=0.01
                        safety_term  = (Lm - Li + ...
                            self.MinSafetyDistance)/self.x_0(2);
                        phi = @(x)(self.ReactionTime+ ...
                            safety_term)*min((x(1)-self.x_0(1))/Li, 1) - ...
                            safety_term;
                        % phi = @(x) self.ReactionTime*x(1)/(x_des-self.x_0(1));
                    else
                        phi = @(x) self.ReactionTime;
                    end
                    % Extract cav c (adj_vehicle)
                    x_k_adj = self.contruct_integrator_states(...
                            self.extract_states_from_id(self.Ego_cav_id));
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des, y_des);
                % CAV C is safe with respect to Leader vehicle (veh U) and
                % CAV 1
                case 'cavC'
                    collab = 1;
                    % Extract adj vehicle
                    adj_cav = self.find_vehicle_from_id(self.Front_cav_id);
                    Lm = adj_cav.x_f - adj_cav.x_0(1);
                    Li = self.x_f - self.x_0(1);
                    if norm(x_k_ego(1)-x_des)>=0.01
                        safety_term  = (Lm - Li + ...
                            self.MinSafetyDistance)/self.x_0(2);
                        phi = @(x)(self.ReactionTime+ ...
                            safety_term)*min((x(1)-self.x_0(1))/Li, 1) - safety_term;
                        % phi = @(x) self.ReactionTime*x(1)/(x_des-self.x_0(1));
                    else
                        phi = @(x) self.ReactionTime;
                    end
                    % Extract cav c (adj_vehicle)
                    x_k_adj = self.contruct_integrator_states(...
                            self.extract_states_from_id(self.Front_cav_id));
                    % Compute desired y Position
                    if ~self.StartLatManeuver 
                        y_des = -y_des;
                    end
                    
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des, y_des, self.StartLatManeuver);
                otherwise
                    error('case not defined')
            end
            if ~status
                error('solution could not be found')
            end
            % -------------- Store Difference between control ----------
            self.u_diff_hist = cat(1,self.u_diff_hist, u_ref-u_k(1));
            % -------------- Apply control input ------------------------
            % Apply constant control input
            x_k = self.Dynamics.integrate_forward(u_k);
            self.CurrentState = self.construct_state_structure(x_k);
            % update Vehicle on DSD
            self.update_dsd_vehicle_states();
            isRunning = status;
            % Update Time Step
            self.CurrentTime = self.CurrentTime + self.SampleTime;
            % ----------Check if maneuver has finished -----------------
            if strcmp(self.RollType,'cavC') && self.StartLatManeuver ...
                    && (self.CurrentState.Position(2)-self.DesYPos)^2<=0.05 ...
                    && isnan(self.LaneChangeTime_end)
                self.LaneChangeTime_end = self.CurrentTime;
            end

            % ------- Check if lateral maneuver should take place --------
            if strcmp(self.RollType,'cavC') && ~self.StartLatManeuver
                % Compute phi function for cav 1
                if x_k_adj(1)-x_k_ego(1)>=self.MinSafetyDistance+5 ...
                        && self.AdaptiveSafetyCheck
                    phi1 = phi(x_k_ego);
                else
                    phi1 = self.ReactionTime;
                end
                
                % Compute phi function for cav 2
                adj2_cav = self.find_vehicle_from_id(self.Rear_cav_id);
                Li2 = adj2_cav.x_f - adj2_cav.x_0(1);
                Lm2 = self.x_f - self.x_0(1);
                x_k_2 = self.contruct_integrator_states(...
                            self.extract_states_from_id(self.Rear_cav_id));
                if x_k_ego(1)-x_k_2(1)>=0 && self.AdaptiveSafetyCheck
                    safety_term  = (Lm2 - Li2 + ...
                        self.MinSafetyDistance)/self.x_0(2);
                    phi2 = (self.ReactionTime+ ...
                        safety_term)*min((x_k_2(1)-self.x_0(1))/Li, 1) - ...
                        safety_term;
                else
                    phi2 = self.ReactionTime;
                end
                self.StartLatManeuver = ...
                    self.check_lateral_maneuver_conditions(phi1, phi2);
                % Store time at which maneuver started
                if self.StartLatManeuver
                    self.LaneChangeTime_start=self.CurrentTime;
                end
            end

        end %step


        function [leader, isLeader] = find_leader(self)
            % Computes leader, if there is no vehicle in front, outputs
            % an empty value for leader and isLeader is true
            % Get vehicle current lane
            egoCurrLane = self.Vehicle.currentLane;
            % Extract Actors that are on the same lane as ego vehicle
            targetVehicles = self.Scenario.Actors;
            targetActors = self.Vehicle.targetPoses;
            % Create an empty cell containing candidate vehicles
            leader = [];
            currMin = inf;
            for i = 1:length(targetVehicles)
                % Extract each vehicle current lane
                targetVehLane = targetVehicles(i).currentLane;
                if targetVehLane == egoCurrLane
                    % Extract vehicle relative to ego vehicle
                    candidateVehicle = targetActors(...
                        [targetActors.ActorID] == targetVehicles(i).ActorID);
                    % Check if empty
                    if isempty(candidateVehicle)
                        continue
                    end
                    % Check if in front of ego vehicle
                    if candidateVehicle.Position(1)>0 && ...
                            candidateVehicle.Position(1) < currMin
                        currMin = candidateVehicle.Position(1);
                        leader = targetVehicles(i);
                    end %if
                end %if
            end %for
            % Check if ego vehicle is leader instead
            if isempty(leader)
                isLeader = true;
            else
                isLeader = false;
            end
        end %find_leader
        function state_hist = get_state_history(self, type)
            % Returns the timeseries datatype of state history
            arguments
                self
                type (1,:) char {mustBeMember(type,{'x','y','v','theta','accel','steering','safety','ocp_difference'})} 
            end
            
            switch type
                case 'x'
                    %Position x
                    time = self.Dynamics.t_hist;
                    vec = self.Dynamics.x_hist(1,:);
                    units = 'm';
                case 'y'
                    %Position y
                    time = self.Dynamics.t_hist;
                    vec = self.Dynamics.x_hist(2,:);
                    units = 'm';
                case 'v'
                    %speed
                    time = self.Dynamics.t_hist;
                    vec = self.Dynamics.x_hist(3,:);
                    units = 'm/s';
                case 'theta'
                    %Heading Angle
                    time = self.Dynamics.t_hist;
                    vec = self.Dynamics.x_hist(4,:)*180/pi;
                    units = 'deg';
                case 'accel'
                    % Acceleration
                    time = self.Dynamics.t_hist(1:end-1);
                    vec = self.Dynamics.u_hist(1,:);
                    units = 'm/s^2';
                case 'steering'
                    % Angular Rate
                    time = self.Dynamics.t_hist(1:end-1);
                    vec = self.Dynamics.u_hist(2,:)*180/pi;
                    units = 'deg';
                case 'safety'
                    %Position x
                    time = self.Dynamics.t_hist;
                    pos = self.Dynamics.x_hist(1,:);
                    speed = self.Dynamics.x_hist(3,:);
                    vec = pos+speed*self.ReactionTime+self.MinSafetyDistance;
                    units = 'm';
                case 'ocp_difference'
                    time = self.Dynamics.t_hist(1:end-1);
                    vec = self.u_diff_hist;
                    units = 'm/s^2';

            end
            % Construct timeseries data
            state_hist = timeseries(vec,time,"Name",type);
            state_hist.TimeInfo.Units = 'sec';
            state_hist.DataInfo.Units = units;

        end
    end %public Methods

    
    methods (Access = protected)
        function update_actor_states(self)
            % Updates the vehicle states on cuboid world and extracts
            % states from vehicle dynamics
            self.CurrentState = self.construct_state_structure(self.Dynamics.x_k);
            % Update States
            self.Vehicle.Position(1:2) = self.CurrentState.Position;
            self.Vehicle.Velocity(1) = self.CurrentState.Velocity;
            self.Vehicle.Yaw = self.CurrentState.Heading*180/pi;
        end    
        function hasStartedLatManeuver = check_lateral_maneuver_conditions(self, phi1, phi2)
            % check_lateral_maneuver_conditions
            % -------------------------------------------------------------
            % monitors the conditions for which a lateral maneuver should
            % take place.
            % =============================================================
            % Sanity checks
            % assert(~self.StartLatManeuver, ...
            %     'Lateral Maneuver has already been started')
            % Make sure that ids are valid
            assert(~isempty(self.Front_cav_id) & ...
                ~isempty(self.Front_cav_id), 'IDs have not been defined')
            % Extract vehicle objects
            veh1 = self.find_vehicle_from_id(self.Front_cav_id);
            veh2 = self.find_vehicle_from_id(self.Rear_cav_id);

            %% Distance between CAV C and CAV1
            % Longitudinal Position Condition
            subCondition1C = (veh1.CurrentState.Position(1)- ... 
                self.CurrentState.Position(1))>=...
                self.CurrentState.Velocity(1)*phi1 + ...
                self.MinSafetyDistance;
            %% Distance between CAV 2 and CAVC
            % Longitudinal Position Condition
            subCondition2C = (self.CurrentState.Position(1)- ... 
                veh2.CurrentState.Position(1))>=...
                veh2.CurrentState.Velocity(1)*phi2 + ...
                self.MinSafetyDistance;            
            % Check if maneuver should start
            hasStartedLatManeuver = subCondition1C && subCondition2C;
        end % check_lateral_maneuver_conditions
    end % Protected Methods
    %% Setters and Getters
    methods
        function leader = get.Leader(self)
            % function to check if leader exists for vehicle. It forces a
            % leader checkup
            [self.Leader, self.IsLeader] = self.find_leader();
            % output values
            leader = self.Leader;
        end
        function isLeader = get.IsLeader(self)
            [self.Leader, self.IsLeader] = self.find_leader();
            % output values
            isLeader = self.IsLeader;
        end
    end

    %% Util Methods (Methods used only by this class)
    methods (Access = private)

       function states = extract_states_from_id(self, id)
            actors = self.Scenario.Actors;
            veh_index = find([actors.Name]== id);
            states.Position = actors(veh_index).Position(1:2)';
            states.Velocity = (actors(veh_index).Velocity(1));
            states.Heading = actors(veh_index).Yaw*pi/180;
       end
       function cav = find_vehicle_from_id(self, id)
            names = arrayfun(@(x) x.VehicleID',self.cav_env,'UniformOutput',false);
            index = find(strcmp(names,id));
            cav = self.cav_env(index);
       end

       function hasBeenUpdated = update_dsd_vehicle_states(self)
            %Updates states on dsd scenario by retrieving states from SUMO
            %and then forcing output on DSD scenario
            % -------------------------------------------------------------
            try
                % Update states from vehicle on scenario
                self.Vehicle.Position(1:2) = self.CurrentState.Position;
                self.Vehicle.Velocity(1) = self.CurrentState.Velocity;
                self.Vehicle.Yaw = self.CurrentState.Heading*180/pi;
                hasBeenUpdated = true;
            catch err
                warning(err.message)
                hasBeenUpdated = false;
            end
       end

        
    end % methods

    methods (Static)
        function state = decompose_state(states)
            state = [states.Position; ...
                    states.Velocity;...
                    states.Heading];
        end
        function state = contruct_integrator_states(state_struct)
            state = [state_struct.Position(1); ...
                     state_struct.Velocity(1)];
        end
        function state = contruct_bicycle_state(state_struct)
            if abs(state_struct.Heading)<=1e-4
                state_struct.Heading = 0;
            end
            state = [state_struct.Position(1:2); ...
                     state_struct.Velocity(1);
                     state_struct.Heading];
        end
        function state_struct = construct_state_structure(states)
            state_struct = struct('Position',states(1:2),...
                'Velocity',states(3),...
                'Heading',states(4));
        end
        % function phi = delta_fun (x_curr, x_des, x_0,tau)
        %     phi = tau*(x_curr-x_0)/(x_des-x_0);
        % end
    end
end


      


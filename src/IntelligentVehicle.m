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
            
        % Rear Safety Distance
        MinSafetyDistance (1,:) double {mustBeNumeric, mustBePositive}= 10;
        ReactionTime (1,:) double {mustBeNumeric, mustBePositive}= 1.2;

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
        % Driving Scenario Designer
        Scenario
        Vehicle
        % Leader Related Variables
        Leader = [];
        IsLeader logical {mustBeNumericOrLogical} = true;
        % Has arrived to destination lane
        HasArrived logical {mustBeNumericOrLogical} = false;
        CAVC_ID;
        CAV1_ID;
        IntersectTime = [];
    end

    properties
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
        % Collaboration vehicles id
        Ego_cav_id = NaN;
        Front_cav_id = NaN;
        Rear_cav_id = NaN;
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
                Options.SampleTime double {mustBeNonnegative, mustBeNumeric} = 0.01;
                Options.VehicleType (1,:) char {mustBeMember(Options.VehicleType,{'NonControlled','CAV'})} = 'NonControlled'
                Options.SafetyDistance (1,:) double {mustBeNumeric, mustBePositive}= 10;
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
            % _________Initialize Vehicle Model ________________
            self.CurrentState = InitialConditions;
            initStates = self.decompose_state(self.CurrentState);
            self.Dynamics = VehicleDynamics(initStates,...
                'dt',self.SampleTime, 't_k', self.CurrentTime);
            % Select mesh depending on class ID
            if Options.VehicleClass == 2
                carMesh = driving.scenario.truckMesh;
            else
                carMesh = driving.scenario.carMesh;
            end
            % _________Initialize Vehicle Model ________________
            self.Vehicle = actor(self.Scenario, ...
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
            % _________Initialize OCP and CBF models_______________
            self.ocp_prob = Control.OCP.CollabAccel(...
                'accelMax', self.AccelMax, ...
                'accelMin', self.AccelMin, ...
                'velMax', self.VelMax, ...
                'velMin', self.VelMin,...
                'tau', self.ReactionTime,...
                'delta_dist', self.MinSafetyDistance);
            self.cbf_prob = Control.CBF.OCBF(...
                'accelMax', self.AccelMax, ...
                'accelMin', self.AccelMin, ...
                'velMax', self.VelMax, ...
                'velMin', self.VelMin,...
                'tau', self.ReactionTime,...
                'delta_dist', self.MinSafetyDistance);
        end %constructor

        
        function hasDefinedCavRoll = define_cav_roll (...
                self, cavType, tf, xf, v_des, e_collab_id,f_collab_id, r_collab_id)
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
                e_collab_id = NaN;
                f_collab_id = NaN;
                r_collab_id = NaN;
            end
            % Sanity Check to make sure that ids have been stablished
            % properly
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
                posU = leader.Position(1);
                velU = norm(self.Leader.Velocity(1:2));
                x_obst_leader = [posU, velU]';
            end
            % Compute Optimal Control Solution
            hasDefinedCavRoll = self.ocp_prob.compute_ocp(...
                x_0_ego, tf, xf, v_des, x_obst_leader);
            
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
                isRunning = true;
                return
            end
            % ----------- Vehicle is collaborating ------------------------
            % Create initial conditions vector
            x_k_ego = self.contruct_integrator_states(self.CurrentState);
            % Extract leader's state (if it exists)
            [self.Leader, self.IsLeader] = self.find_leader();
            if self.IsLeader
                x_k_lead = x_k_ego;
                x_k_lead(1) = x_k_lead(1) + 1000;
            else
                posU = leader.Position(1);
                velU = norm(self.Leader.Velocity(1:2));
                x_k_lead = [posU, velU]';
            end
            % Compute u_reference function based on current state
            [~, v_ref, u_ref] = self.ocp_prob.extract_cntrl_input(...
                x_k_ego, self.CurrentTime);
            % Compute remaining time from terminal time
            t_des = max(self.t_f - (self.CurrentTime - self.t_0), 1);
            x_des = self.x_f;
            % Compute CBF control input based on reference signal    
            switch self.RollType
                % CAV 1 is only safe with respect to vehicle in front
                case 'cav1'
                    collab = 0;
                    phi = 0;
                    x_k_adj = x_k_ego*0;
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des);
                % CAV 1 is safe with respect to Leader vehicle (cav 1 and
                % cav C terminal position
                case 'cav2'
                    collab = 1;
                    phi = self.delta_fun(x_k_ego(1), x_des, self.x_0, 1.2);
                    % Extract cav c (adj_vehicle)
                    x_k_adj = self.contruct_integrator_states(...
                            self.extract_states_from_id(self.CAVC_ID));
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des);
                % CAV C is safe with respect to Leader vehicle (veh U) and
                % CAV 1
                case 'cavC'
                    collab = 1;
                    phi = self.delta_fun(x_k_ego(1), x_des, self.x_0, 1.2);
                    % Extract cav c (adj_vehicle)
                    x_k_adj = self.contruct_integrator_states(...
                            self.extract_states_from_id(self.CAV1_ID));
                    [status, u_k] = self.cbf_prob(collab, ...
                        x_k_ego, x_k_lead, x_k_adj, u_ref, v_ref, ...
                        phi, t_des, x_des);
                otherwise
                    error('case not defined')
            end
            if ~status
                error('solution could not be found')
            end
            % -------------- Apply control input ------------------------
            % Apply constant control input
            x_k = self.Dynamics.integrate_forward(u_k);
            self.CurrentState = self.construct_state_structure(x_k);
            % update Vehicle on DSD
            self.update_dsd_vehicle_states();
            isRunning = true;
            % Update Time Step
            self.CurrentTime = self.CurrentTime + self.SampleTime;
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
            allPoses = actorPoses(self.Scenario);
            veh_index = find([allPoses.ActorID]== id);
            states.Position = allPoses(veh_index).Position(1:2)';
            states.Velocity = norm(allPoses(veh_index).Velocity(1:2));
            states.Heading = allPoses(veh_index).Yaw*pi/180;
           
        end

       function hasBeenUpdated = update_dsd_vehicle_states(obj)
            %Updates states on dsd scenario by retrieving states from SUMO
            %and then forcing output on DSD scenario
            % -------------------------------------------------------------
            try
                % Update states from vehicle on scenario
                obj.Vehicle.Position(1:2) = obj.CurrentState.Position;
                obj.Vehicle.Velocity(1:2) = obj.CurrentState.Velocity;
                obj.Vehicle.Yaw = obj.CurrentState.Heading*180/pi;
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
                     norm(state_struct.Velocity(1))];
        end
        function state_struct = construct_state_structure(states)
            state_struct = struct('Position',states(1:2),...
                'Velocity',states(3),...
                'Heading',states(4));
        end
        function phi = delta_fun (x_curr, x_des, x_0,tau)
            phi = tau*(x_curr-x_0)/(x_des-x_0);
        end
    end
end


        % function isRunning = step(self)
        %     % Performs an update step and increases time step by one.
        %     % Additionally, it updates vehicle position on DSD scenario and
        %     % behaves according to current behaviour type
        %     % -------------------------------------------------------------
        %     if self.CurrentTime <= self.StopTime
        %         % Find behaviour
        %         isDone = false;
        %         switch self.DrivingBehaviour
        %             case 'collaborating'
        %                 % Collaborating State
        %                 [states, isDone]= self.Dynamics.update();
        %                 if isDone
        %                     % Set Has arrived variable to true
        %                     self.HasArrived = true;
        %                 end
        %             otherwise % selfless
        %                 % Apply current Speed
        %                 states = self.Dynamics.update_state();
        %         end
        %         % update states
        %         self.CurrentState = self.construct_state_structure(states);
        %         % update Vehicle on DSD
        %         self.update_dsd_vehicle_states();
        %         % Update Time Step
        %         self.CurrentTime = self.CurrentTime + self.SampleTime;
        %         % update state for output
        %         if ~isDone
        %             isRunning = true;
        %         else
        %             isRunning = false;
        %         end
        %     else
        %         isRunning = false;
        %         warning('Simulation has reached defined final time')
        %     end
        % 
        % end %step


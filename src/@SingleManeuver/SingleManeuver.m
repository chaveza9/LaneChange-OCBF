classdef SingleManeuver < matlab.System
    %SINGLEMANEUVER Object that contains logic for performing lane-change
    %maneuver (longitudinal and lateral) 
    %   Contains logic that calls collaboration selectors, CAV's roll
    %   definition, and step-by-step calling of update functions for every
    %   single vehicle involved in the collaboration maneuver
    
    properties (Access=public)
        % Vehicles physical constraints structure
        Constraints = struct('u_max',3.3,'u_min',-7,'v_max',33,'v_min',10)
        % Plot Trajectories for all estimations
        Display {mustBeNumericOrLogical} = false
        % Plot Trajectories for only Results
        DisplayResults {mustBeNumericOrLogical} = false
        % Create Results Log File
        ResultsLog {mustBeNumericOrLogical} = false
        % Results log file id
        ResultsFileID {mustBeNumeric} = -1
        % Create Estimations Log File
        EstimationLog {mustBeNumericOrLogical} = false  
        % Estimation log file id
        EstimationFileID {mustBeNumeric} = -1
        % Verbose results on console
        Verbose {mustBeNumericOrLogical} = false
        % Min delta for lateral initialization [m]
        DeltaLong = 5;
        % Min Tolerance speed for cav C lateral maneuver [m/s]
        DeltaSpeedLat = 4;
    end
    properties (GetAccess=public, SetAccess=private)
        %% Input members
        % Driving Scenario
        Scenario 
        % Structure of type IntelligentVehicle constaining candidate Set 
        CavCandidates %{mustBeUnderlyingType(CavCandidates,'IntelligentVehicle')}
        % Ego Vehicle
        CavC %{mustBeUnderlyingType(CavC,'IntelligentVehicle')}
        % Uncontrolled Vehicle
        VehicleU %{mustBeUnderlyingType(VehicleU,'IntelligentVehicle')}
        %% Selected Collaboration Results
        % CAV 1 Vehicle (Can be empty)
        Cav1 = []
        % CAV 2 Vehicle (Can be empty)
        Cav2 = []
        % Collaboration Selector type
        CollabSelectorType (1,:) char {mustBeMember(CollabSelectorType,{'Social','Selfish','SelfishRelaxed','invalid'})} = "invalid" 
        % Collaboration type case
        CollabType = [];
        % Maneuver Lenght [s]
        TerminalTime (1,1) {mustBeNumeric} = 0;
        % Terminal Positions Structure [m]
        TerminalPosition = struct('veh1',[],'veh2',[],'vehC',[]);
        % Disruption [m^2]
        TotDisruption (1,1) {mustBeNumeric} = inf;
        % energy [m/s^2]
        TotalEnergy (1,1) {mustBeNumeric} = inf;
        % Flow Speed [m/s]
        FlowSpeed (1,1) {mustBeNumeric} = 33;
        % Is Collaboration Feasible
        IsFeasible (1,1) {mustBeNumericOrLogical} = false;
        % Is Maneuver Done
        IsManeuverFinished {mustBeNumericOrLogical} = false;

    end
    properties(Access=private)
        % Collaboration Selector objects
        SelfishCollabSelector 
        SocialCollabSelector 
        %% Helper properties
        HasStartedLongManeuver {mustBeNumericOrLogical} = false;
        HasStartedLatManeuver {mustBeNumericOrLogical} = false;

    end
    
    methods
        function obj = SingleManeuver(cavC, vehicleU, cavCandidates,...
                scenario,  varargin)
            %SINGLEMANEUVER Constructs an instance of this class
            % -------------------------------------------------------------
            %  Defines the type of collaboration selection chosen by the
            %  user and initialized single maneuver computation given user
            %  properties
            % =============================================================
            %% Propulate Properties
            % Check argument validity
            mustBeUnderlyingType(cavC,'IntelligentVehicle')
            mustBeUnderlyingType(vehicleU,'IntelligentVehicle')
            mustBeUnderlyingType(cavCandidates,'IntelligentVehicle')
            mustBeUnderlyingType(scenario,'drivingScenario')
     
            obj.CavC = cavC;
            obj.VehicleU = vehicleU;
            obj.CavCandidates = cavCandidates;
            obj.Scenario = scenario;
            % Setup variable public properties
            if nargin > 4
                setProperties(obj,nargin-4,varargin{:});
            end
            %% Initialize Collaboration Selectors
            % Selfish
            obj.SelfishCollabSelector = ...
                CAV.SelfishCollaborationSelection(cavC, vehicleU, cavCandidates);
            if nargin > 4
                setProperties(obj.SelfishCollabSelector,nargin-4,varargin{:});
            end
            % Social
            obj.SocialCollabSelector = ...
                CAV.SocialCollaborationSelection(cavC, vehicleU, cavCandidates);
            if nargin > 4
                setProperties(obj.SocialCollabSelector,nargin-4,varargin{:});
            end
        end %SingleManeuver
        
        function [isFeasible,  cav1, cav2, cavC, isCollabReq] = ...
                find_optimal_pair(obj)
            %find_optimal_pair 
            % -------------------------------------------------------------
            %   Finds the optimal collaboration pair and finds the maneuver
            %   type ('social','selfish') with minimum disruption
            % -------------------------------------------------------------
            % OUTPUT
            % - isFeasible: (bool) boolean stating if a collaboration pair 
            %   was found 
            % - cav1: (IntelligentVehicle/[]) collaborating vehicle 1
            % - cav2: (IntelligentVehicle/[]) collaborating vehicle 2
            % - cavC: (IntelligentVehicle) collaborating vehicle C
            % - isCollabReq: (bool) Outputs decision variable on whether 
            %   collaboration is required (candidate set is empty and 
            %   cavC does not require any collaboration to perform maneuver
            % - maneuverType: (string) {'Social','Selfish','invalid'}
            % =============================================================
            % Placeholder variables
            disruptionSocial = inf;
            disruptionSelfish = inf;
            cav1 = [];
            cav2 = [];
            cavC = obj.CavC;
            isFeasible = true;
            %% Compute Selfish Maneuver 
            [isSelfishFeasible, isCollabReq, cav1_sel, cav2_sel,...
                ~, collabType_sel] = ...
                obj.SelfishCollabSelector.find_optimal_collaboration_triplet(obj.Verbose);
            % Check if collaboration is required at all
            if ~isCollabReq
                return
            end 
            % Extract disruption if maneuver is feasible
            if isSelfishFeasible
                disruptionSelfish = obj.SelfishCollabSelector.TotDisruption;
            end
            %isSelfishFeasible = 0;   
            %% Compute Social Maneuver 
%             [isSocialFeasible, isCollabReq, cav1_soc, cav2_soc,...
%                ~, collabType_soc] = ...
%                obj.SocialCollabSelector.find_optimal_collaboration_triplet(obj.Verbose);
%             % Check if collaboration is required at all
%             if ~isCollabReq
%                 return
%             end 
%             % Extract disruption if maneuver is feasible
%             
%             if isSocialFeasible
%                 disruptionSocial = obj.SocialCollabSelector.TotDisruption;
%             end
             isSocialFeasible = 0;
            %% Determine optimal collaboration strategy option
            isFeasible = isSocialFeasible || isSelfishFeasible;
            obj.IsFeasible = isFeasible;
            % if no maneuver was feasible return
            if ~isFeasible
                return
            end
            
            % Find maneuver collaboration type with minimum disruption
            if disruptionSocial >= disruptionSelfish
                % Populate object properties
                obj.Cav1 = cav1_sel;
                obj.Cav2 = cav2_sel;
                obj.CollabType = collabType_sel;
                obj.TotDisruption = disruptionSelfish;
                obj.TotalEnergy = obj.SelfishCollabSelector.TotalEnergy;
                obj.TerminalTime = obj.SelfishCollabSelector.TerminalTime;
                obj.FlowSpeed = obj.SelfishCollabSelector.FlowSpeed;
                obj.TerminalPosition = obj.SelfishCollabSelector.TerminalPosition;
                obj.CollabSelectorType = obj.SelfishCollabSelector.CavCType;
                
            else
                % Populate object properties
                obj.Cav1 = cav1_soc;
                obj.Cav2 = cav2_soc;
                obj.CollabType = collabType_soc;
                obj.TotDisruption = disruptionSocial;
                obj.TotalEnergy = obj.SocialCollabSelector.TotalEnergy;
                obj.TerminalTime = obj.SocialCollabSelector.TerminalTime;
                obj.FlowSpeed = obj.SocialCollabSelector.FlowSpeed;
                obj.TerminalPosition = obj.SocialCollabSelector.TerminalPosition;
                obj.CollabSelectorType = obj.SocialCollabSelector.CavCType;
            end
            
            if obj.Verbose
                % Display Terminal time and positions solutions
                fprintf('*********************************************\n');
                fprintf(' SOLUTION\n')
                fprintf('---------------------------------------------\n');
                fprintf('Maneuver Type: %s \n',obj.CollabSelectorType);
                fprintf('---------------------------------------------\n');
                fprintf('Vehicle1\tVehicle2\tVehicleC \n')
                fprintf('Collaboration Type = %d \n',obj.CollabType);
                fprintf('Total Energy = %f \n',obj.TotalEnergy);
                fprintf('Disruption = %f \n',obj.TotDisruption);
                fprintf('Terminal Time = %f \n',obj.TerminalTime);
                fprintf('Desired Speed = %f [m/s]\n', obj.FlowSpeed);
                fprintf('Terminal Position\n');
                fprintf('Vehicle1\tVehicle2\tVehicleC \n')
                fprintf('%f\t%f\t%f\n',obj.TerminalPosition.veh1, ...
                    obj.TerminalPosition.veh2,...
                    obj.TerminalPosition.vehC);
                fprintf('*********************************************\n');
            end
            % Populate optimal collaboration pairs output
            cav1 = obj.Cav1;
            cav2 = obj.Cav2;

        end %find_optimal_pair
        function hasStarted = start_longitudinal_maneuver(obj)
            % start_longitudinal_maneuver
            % -------------------------------------------------------------
            % Defines rolls for every vehicle in the candidate set,
            % including non-optimal vehicles, and computes planning
            % solutions as a reference trajectory for every vehicle inside
            % the collaboration set
            % =============================================================
            % Define maneuver rolls
            hasDefinedRolls = obj.define_maneuver_rolls;
            % Start longitudinal maneuver computation
            hasStartedLongMan = obj.compute_longitudinal_solution;
            % Update properties 
            hasStarted = hasDefinedRolls && hasStartedLongMan;
            obj.HasStartedLongManeuver = hasStarted;
        end % start_longitudinal_maneuver

        function isRunning = update(obj)
            %update
            % -------------------------------------------------------------
            % Computes a single step trajectory for every vehicle in the
            % candidate set
            % =============================================================
            % Update Vehicles
            isRunning = obj.CavC.step();
            isRunning = isRunning && obj.VehicleU.step();
            for i=1:length(obj.CavCandidates)
                isRunning = obj.CavCandidates(i).step();
            end
            % Advance on scenario
            advance(obj.Scenario);
            % Check if lateral maneuver should start
            obj.HasStartedLatManeuver = obj.check_lateral_maneuver_conditions;
            % if has started lat maneuver, end longitudinal maneuver
            if obj.HasStartedLatManeuver
                isRunning = false;
            end
        end
    end

    methods(Access=private)
        function hasStartedLatManeuver = check_lateral_maneuver_conditions(obj)
            % check_lateral_maneuver_conditions
            % -------------------------------------------------------------
            % monitors the conditions for which a lateral maneuver should
            % take place.
            % =============================================================
            % Sanity checks
            assert(obj.HasStartedLongManeuver, ...
                'Longitudinal Maneuver has not been computed yet')
            assert(~obj.HasStartedLatManeuver, ...
                'Lateral Maneuver has already been started')
            veh1 = obj.Cav1;
            veh2 = obj.Cav2;
            vehC = obj.CavC;
            vehU = obj.VehicleU;
            % Variable placeholderds
            subConditions1 = 1;
            subConditions2 = 1;
            delta = obj.DeltaLong;
            % Get vehicles involved in maneuver
            if ~isempty(veh1) && isempty(veh2)
                veh2 = veh1.find_follower;
            elseif isempty(veh1) && ~isempty(veh2)
                veh1 = veh2.find_leader;
            elseif isempty(veh1) && isempty(veh2)
                error('Vehicles not found, maneuver check is unfeasible')
            end
            % Extract Minimum safe distances
            % Compute front overhangs
            vehC_FrontOverhang = vehC.Vehicle.Length - ...
                vehC.Vehicle.RearOverhang;
            if ~isempty(veh2)
                if isUnderlyingType(veh2,'driving.scenario.Vehicle')
                    veh2_FrontOverhang = veh2.Length-veh2.RearOverhang;
                else
                    veh2_FrontOverhang = veh2.Vehicle.Length - ...
                        veh2.Vehicle.RearOverhang;
                end
            end
            %% CAV C conditions
            % Speed Conditions
            speedCondition = ...
                (vehC.CurrentState.Velocity(1) - obj.FlowSpeed)^2 <= ...
                obj.DeltaSpeedLat^2;
            % Check if Speed condition  should be voided (cav 2 is too far
            % behind)
            if ~isempty(veh2)
                if vehC.CurrentState.Position(1)-vehC.Vehicle.RearOverhang -...
                        (veh2.CurrentState.Position(1) + veh2_FrontOverhang)>= ...
                        obj.Constraints.v_max*0.6+obj.DeltaLong*7
                    speedCondition = true;
                end
            end
            % Longitudinal Position Condition
            subConditionC = ((vehU.CurrentState.Position(1)-vehU.Vehicle.RearOverhang) ...
                - (vehC.CurrentState.Position(1)+vehC_FrontOverhang)>= delta...
                && speedCondition)||  ...                
               (vehU.CurrentState.Position(1)-vehU.Vehicle.RearOverhang) ...
                - (vehC.CurrentState.Position(1)+vehC_FrontOverhang)< delta -1;
            
            %% CAV 1 conditions
            if ~isempty(veh1) && ~isUnderlyingType(veh1, 'double')
                subConditions1 = veh1.CurrentState.Position(1) - ...
                    vehC.CurrentState.Position(1) >= delta+vehC_FrontOverhang+...
                    veh1.Vehicle.RearOverhang;    %x1 - xc >= Lv
            end
            %% CAV 2 conditions
            if ~isempty(veh2)
                if isUnderlyingType(veh2,'driving.scenario.Vehicle')
                    subConditions2 = vehC.CurrentState.Position(1) - ...
                     veh2.Position(1) >= delta+veh2_FrontOverhang+...
                     vehC.Vehicle.RearOverhang; %xc - x2 >= Lv
                else
                    subConditions2 = vehC.CurrentState.Position(1) - ...
                     veh2.CurrentState.Position(1) >= delta+veh2_FrontOverhang+...
                     vehC.Vehicle.RearOverhang; %xc - x2 >= Lv
                end
            end

            % Check if maneuver should start
            hasStartedLatManeuver = subConditionC && ...
                subConditions1 && subConditions2;
        end % check_lateral_maneuver_conditions

        function hasStartedLongMan = compute_longitudinal_solution(obj)
            % compute_longitudinal_solution
            % -------------------------------------------------------------
            % Computes a numerical solution for every CAV involved
            % in the collaboration set (if rolls have been specified
            % previously). This solution is used as the reference
            % trajectory planning set that every vehicle should follow
            % =============================================================
            mustBeMember(obj.CollabSelectorType,...
                {'Social','Selfish','SelfishRelaxed'});
            % Placeholder variables
            isFeasible1 = true;
            isFeasible2 = true;
            % Compute CAV C trajectory
            isFeasibleC = obj.CavC.start_longitudinal_maneuver (...
                obj.TerminalTime, obj.TerminalPosition.vehC,...
                'display',obj.DisplayResults);
            % Compute CAV 1 Trajectory
            if ~isempty(obj.Cav1) && any(obj.CollabType == [1,2,3,4]) 
                if strcmp(obj.Cav1.VehicleType,'CAV')
                    isFeasible1 = obj.Cav1.start_longitudinal_maneuver (...
                    obj.TerminalTime, obj.TerminalPosition.vehC,...
                    'display',obj.DisplayResults);
                end
            end
            % Define CAV 2 roll
            if ~isempty(obj.Cav2) && any(obj.CollabType == [1,2,5]) 
                if strcmp(obj.Cav2.VehicleType,'CAV')
                    isFeasible2 = obj.Cav2.start_longitudinal_maneuver (...
                    obj.TerminalTime, obj.TerminalPosition.vehC,...
                    'display',obj.DisplayResults);
                end
            end
            % Check if maneuver was feasible
            hasStartedLongMan = isFeasibleC && isFeasible1 && isFeasible2;
           
        end % compute_longitudinal_solution
        function hasDefinedRolls = define_maneuver_rolls(obj)
            % define_maneuver_rolls
            % -------------------------------------------------------------
            % Initializes the rolls for every optimal pair in the
            % collaboration set. Additionally, it sets vehicles with
            % non-collaborating roll to ACC mode. When defining rolls,
            % each vehicle defines a parametric numerical model for 
            % the solution of its corresponding optimal control problem
            % =============================================================
            % Check if roll can be defined
            mustBeMember(obj.CollabSelectorType, {'Social','Selfish','SelfishRelaxed'})
                
            % Define CAV C roll based on collaboration strategy
            hasDefinedRolls = obj.CavC.define_cav_roll(obj.CollabSelectorType, ...
                'cavC', obj.FlowSpeed);
            % Define CAV 1 roll
            if ~isempty(obj.Cav1) && any(obj.CollabType == [1,2,3,4]) 
                if strcmp(obj.Cav1.VehicleType,'CAV')
                    hasDefinedRolls = hasDefinedRolls && ...
                        obj.Cav1.define_cav_roll('Accel', ...
                    'cav1', obj.FlowSpeed);
                end
            end
            % Define CAV 2 roll
            if ~isempty(obj.Cav2) && any(obj.CollabType == [1,2,5]) 
                if strcmp(obj.Cav2.VehicleType,'CAV')
                    hasDefinedRolls = hasDefinedRolls && ...
                        obj.Cav2.define_cav_roll('Decel', ...
                    'cav2', obj.FlowSpeed);
                end
            end
            % Command ACC for non-cooperative vehicles
            if hasDefinedRolls && obj.IsFeasible 
                obj.set_non_optimal_candidates_to_acc;
            end
        end % define_maneuver_rolls
        function set_non_optimal_candidates_to_acc(obj)
            % set_non_optimal_candidates_to_acc
            % -------------------------------------------------------------
            % Sets non optimal candidates to ACC state
            % =============================================================
            % check that CAV1 and CAV2 have already been selected based on
            % collaboration type
            assert(~isempty(obj.CollabType),...
                'feasible collaboration set Should be determined before calling function')
            if ~isUnderlyingType(obj.Cav1,'IntelligentVehicle') ||...
                    ~strcmp(obj.Cav1.VehicleType,'CAV')
                cav1_condition = 0;
            else
                cav1_condition = obj.Cav1;
            end
            if isempty(obj.Cav2) || ...
                    ~isUnderlyingType(obj.Cav2,'IntelligentVehicle')  
                cav2_condition = 0;
            else
                cav2_condition = obj.Cav2;
                % Activate HDV2 if required
                if ~strcmp(obj.Cav2.VehicleType,'CAV') && ...
                        strcmp(obj.Cav1.VehicleType,'CAV')
                    cav2_condition = 0;
                end
            end
            % Command ACC
            for i = 1:length(obj.CavCandidates)
                if obj.CavCandidates(i)~=cav1_condition && ...
                        obj.CavCandidates(i)~=cav2_condition
                    obj.CavCandidates(i).abort_collaboration;
                end
            end
        end %set_non_optimal_candidates_to_acc
    end
end


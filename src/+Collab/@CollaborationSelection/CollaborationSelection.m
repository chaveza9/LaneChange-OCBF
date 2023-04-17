classdef CollaborationSelection < matlab.System
    % DefineCollaborationVeh 
    % ---------------------------------------------------------------------
    % Extract current conditions from scenario and defines CAVs using their
    % respective classes. Defines cavs based on a group of four vehicles.
    % However, the output can be a set of {cavC, cav1, cav2}, {cavC, cav1},
    % or {cavC, cav1}. Additionally, a set of initial conditions is 
    % computed based on feasibility
    % =====================================================================

    properties (SetAccess = protected, GetAccess = public)
        %% Input members
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
        % Collaboration type case
        CollabType = [];
        % Maneuver Lenght [s]
        TerminalTime (1,1) {mustBeNumeric} = 0;
        % Terminal Positions Structure [m]
        TerminalPosition = dictionary;
        % Disruption [m^2]
        TotDisruption (1,1) {mustBeNumeric} = inf;
        % Flow Speed [m/s]
        FlowSpeed (1,1) {mustBeNumeric} = 34;
        % Is Collaboration Feasible
        IsFeasible (1,1) {mustBeNumericOrLogical} = false;
    end
    properties (Access=public)
        % Number of workers for paralle computation of trajectories (set
        % value to 0 for serial computation and disable parallel)
        Workers = 5; 
        % Number of desired relaxations per candidate pair
        relaxationTimes = 10;   
        % Vehicles physical constraints structure
        Constraints = struct('u_max',3.3,'u_min',-7,'v_max',33,'v_min',10)
        % Verbose
        Verbose {mustBeNumericOrLogical} = false
        % Minimum Delta safety distance [m]
        DeltaSafe {mustBeNumeric} = 4.3 
        % Reaction Time [s]
        ReactionTime = 1.2
        % Weight Disruption DeltaX
        AlphaDeltaX = 0.8
        % Weight Disruption Terminal Speed
        AlphaTerminalSpeed = 0.2
        % Front CAV U Distance [m]
        DeltaFrontRelDistU = 10
        % DesiredSpeedType
        DesSpeedType {mustBeMember(DesSpeedType,{'Average','Fixed'})} = 'Average'
        % Weights for Average Speed consideration [avg, max_speed]
        DesSpeedWeights = [0.5, 0.5]
    end

    methods
        function obj = CollaborationSelection(cavC, vehicleU, ...
                cavCandidates, varargin)
            % CollaborationSelection
            % -------------------------------------------------------------
            %   Initialized class for Collaboration Selection. 
            % =============================================================  
            mustBeUnderlyingType(cavC,'IntelligentVehicle')
            mustBeUnderlyingType(vehicleU,'IntelligentVehicle')
            mustBeUnderlyingType(cavCandidates,'IntelligentVehicle')
            % Propulate Mandatory Properties
            obj.CavC = cavC;
            obj.VehicleU = vehicleU;
            % Extract Vehicel Poses
            candidatePoses = cell2mat(arrayfun(@(x) x.CurrentState.Position',...
                cavCandidates, 'UniformOutput',false)');
            % order vehicles in descending order
            [~,candidateIndexes] = sort(candidatePoses(:,1),'descend');
            cavCandidates = cavCandidates(candidateIndexes);
            % extract Vehicle U pose
            vehicleUPose = obj.extract_vehicle_states(vehicleU);
            refPose = vehicleUPose.Pose(1) + vehicleU.Vehicle.RearOverhang + obj.DeltaFrontRelDistU;
            % Substract reference pose from vector of candidates
            translatedPoses = candidatePoses(:,1)-refPose;
            cavCandidates = cavCandidates(translatedPoses <=0);
            % Populate CAV candidates
            obj.CavCandidates = cavCandidates;
            % Setup variable public properties
            if nargin > 3
                setProperties(obj,nargin-3,varargin{:});
            end

        end

        function [isFeasible, isCollabReq, cav1, cav2, cavC, collabType] = ...
                find_optimal_collaboration_triplet(obj, verbose)
            % find_optimal_collaboration_triplet
            % -------------------------------------------------------------
            % Finds the optimal collaboration pair (if there is any) by
            % iterating through every candidate pair and estiating the
            % solution for the pair. The optimal candidate pair is selected
            % as the feasible pair with minimum disruption solution
            % -------------------------------------------------------------
            % Inputs:
            % verbose: (bool) (optional) determines if solutions and
            % warning should be displayed on the console
            % -------------------------------------------------------------
            % Outputs:
            % isFeasible: (bool) Flag that indicates if a feasible pair was
            %   found
            % isCollabReq: (bool) Flag that indicates if CAVC requires any
            %   collaboration at all
            % cav1: (IntelligentVehicle/Empty Array) optimal cav 1 (can be
            %   empty depending on collaboration type)
            % cav2: (IntelligentVehicle/Empty Array) optimal cav 2 (can be
            %   empty depending on collaboration type)
            % cavC: (IntelligentVehicle) optimal cav C 
            % collabType: (Integer) Integer between 1 and 5 describing the 
            %   type of collaboration. {1,2: CAV1 and CAV2; 3,4: CAV1;    
            %   5: CAV2} For cases 3,4 and 5, Veh1 or Veh2 can be non
            %   existent or HDVs
            % =============================================================
            arguments
                obj
                verbose (1,2) {mustBeNumericOrLogical} = false
            end
            % Compute Feasible collaboration set solution
            [cavCandidatesSol, isCollabReq] = ...
                obj.compute_feasible_collaboration_set();
            % Extract solution 
            if ~isempty(cavCandidatesSol)
                isFeasible = obj.extract_optimal_pair(...
                    cavCandidatesSol,verbose);
            else
                if verbose
                    warning('Maneuver is infeasible, has been cancelled')
                end
                isFeasible = false;
            end
            % Populate output values
            cav1 = obj.Cav1;
            cav2 = obj.Cav2;
            cavC = obj.CavC;
            collabType = obj.CollabType;
            % Populate feasibility property
            obj.IsFeasible = isFeasible;

        end


    end
    methods(Access = protected)
        
        function isFeasible = extract_optimal_pair(obj, cavCandidatesSol, verbose)
            % extract_optimal_pair
            % -------------------------------------------------------------
            % Extracts collaboration pair with minimal disruption.
            % Additionally, if verbose is set to true during
            % initialization, this function will print maneuver parameters
            % on console for optimal collaboration set. If no optimal pair
            % is found, it will output a flag indicating that maneuver is
            % infeasible
            % =============================================================
            % Check if any maneuver was feasible
            feasManeuvers = cavCandidatesSol(~isinf([cavCandidatesSol.TotalEnergy]));
            if isempty(feasManeuvers)
                if verbose
                    warning('Maneuver is infeasible, has been cancelled')
                end
                isFeasible = false;
                return
            else
                isFeasible = true;
            end
            % Extract index with minimum Disruption
            [~, minIndex] = min([feasManeuvers.Disruption]);
            %% Extract Optimal Pairs
            % CAV 1
            if isUnderlyingType(feasManeuvers(minIndex).cav1Sol.CAV,...
                    'IntelligentVehicle')
                obj.Cav1 = feasManeuvers(minIndex).cav1Sol.CAV;
            end
            % CAV 2
            if isUnderlyingType(feasManeuvers(minIndex).cav2Sol.CAV,...
                    'IntelligentVehicle')
                obj.Cav2 = feasManeuvers(minIndex).cav2Sol.CAV;                
            end

            %% Set CAV 2 leader for car following model
            if ~isempty(obj.Cav2) && ~isempty(obj.Cav1)
                if ~strcmp(obj.Cav2.VehicleType,'CAV') && ...
                        strcmp(obj.Cav1.VehicleType,'CAV')                    
                        obj.Cav2.set_HDV2_leaders(...
                            obj.Cav1.Vehicle.ActorID,...
                            obj.CavC.Vehicle.ActorID, ...
                            feasManeuvers(minIndex).cav2Sol.IntersectTime);
                end
            end
            %% Maneuver Parameters    
            obj.TerminalTime = feasManeuvers(minIndex).TerminalTime;
            obj.TerminalPosition = feasManeuvers(minIndex).TerminalPos;            
            obj.CollabType = feasManeuvers(minIndex).CollabType;
            obj.TotalEnergy = feasManeuvers(minIndex).TotalEnergy;
            obj.CavCType = feasManeuvers(minIndex).cavCSol.ManeuverType;
            obj.TotDisruption = feasManeuvers(minIndex).Disruption;
            obj.FlowSpeed = feasManeuvers(minIndex).DesSpeed;
            %% Create log results
            if obj.ResultsLog
                obj.populate_esimation_log(feasManeuvers(minIndex),...
                    obj.ResultsFileID)
            end
            %% Print Results on Console 
            if verbose
                % Display Terminal time and positions solutions
                fprintf('=============================================\n');
                fprintf(' Number Candidates found: \t %d \n', ...
                    length(cavCandidatesSol))
                fprintf('---------------------------------------------\n');
                fprintf('Maneuver Type: %s \n',obj.CavCType);
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
                fprintf('=============================================\n');
            end
            
        end
        function [cavCandidatesSol, isCollabReq] = ...
                compute_feasible_collaboration_set(obj)
            % compute_feasible_collaboration_set
            % -------------------------------------------------------------
            % Computes a set of solutions based on the iteration of each
            % candidate pair inside the candidate set. It populates the
            % array cavCandidatesSet for every feasible candidate pair
            % =============================================================
            % Create empty array for storing solution set
            cavCandidatesSol = [];
            % Compute the number of vehicles inside candidate set
            numVehicles = length(obj.CavCandidates);
            % Create util variables
            isSetEmpty =  false;
            % Iterate Through candidate set
            if numVehicles < 1
                isSetEmpty = true;
            elseif numVehicles == 1
                % Extract candidate CAV1  
                cav1Candidate = obj.CavCandidates;
                cav2Candidate = [];
               if strcmp(cav1Candidate.VehicleType,'CAV')
                    % Estimate trajectory
                    [isFeasible, maneuverEstimate] = ...
                                obj.estimate_trajectory(...
                                cav1Candidate, cav2Candidate);
                    % Append Solution
                    cavCandidatesSol = obj.appendSol...
                    (isFeasible, maneuverEstimate, cavCandidatesSol);        
                end
            else
                % Compute Flow Speed as the average speed among all cav
                % candidates
                if strcmp(obj.DesSpeedType,'Average')
                    cavVelocity = cell2mat(arrayfun(@(x) x.CurrentState.Velocity',...
                        obj.CavCandidates, 'UniformOutput',false)');
                    avgSpeed = mean(cavVelocity(:,1));
                    desSpeed = sum([avgSpeed, obj.FlowSpeed].*...
                        obj.DesSpeedWeights/sum(obj.DesSpeedWeights));
                else
                    desSpeed = obj.FlowSpeed;
                end
                for i = 1:numVehicles -1
                    % Extract candidate CAV1  
                    cav1Candidate = obj.CavCandidates(i);
                    % Extract candidate CAV2
                    cav2Candidate = obj.CavCandidates(i+1);
                    % Exclude vehicles with intial speed less than
                    % minimum speed
                    if cav2Candidate.CurrentState.Velocity(1) < obj.MinSpeed
                        continue
                    end
                    % Maneuver Type logics
                    if ~strcmp(cav1Candidate.VehicleType,'CAV') && ...
                            ~strcmp(cav2Candidate.VehicleType,'CAV')
                        [isFeasible, maneuverEstimate] = ...
                            obj.estimate_trajectory(...
                            cav1Candidate, cav2Candidate, desSpeed);
                        % Append Solution
                        cavCandidatesSol = obj.appendSol...
                        (isFeasible, maneuverEstimate, cavCandidatesSol);
                    else
                        % Estimate trajectory
                        [isFeasible, maneuverEstimate] = ...
                            obj.estimate_trajectory(...
                            cav1Candidate, cav2Candidate, desSpeed);
                        % Append Solution
                        cavCandidatesSol = obj.appendSol...
                        (isFeasible, maneuverEstimate, cavCandidatesSol);
                    end
                end
            end
            isCollabReq = ~isSetEmpty;
        end % compute_feasible_collaboration_set

 
    end

    methods(Static, Access = protected)
        function states = extract_vehicle_states(car)
            % -------------------------------------------------------------
            % Extracts states from IntelligentVehicle car
            % =============================================================
            arguments
                car {mustBeUnderlyingType(car,'IntelligentVehicle')}       
            end
            states.Pose = [car.CurrentState.Position;
                    car.CurrentState.Velocity;
                    car.CurrentState.Heading;
                    car.CurrentState.Steering]';
            states.Length = car.Vehicle.Length;
            states.RearOverhang = car.Vehicle.RearOverhang;
            states.FrontOverhang =  states.Length-states.RearOverhang;
            states.Collaborating = strcmp(car.VehicleType,'CAV');
        end % extract_vehicle_states
        
        function states = extract_vehicle_states_dsd(car)
            % -------------------------------------------------------------
            % Extracts states from driving.scenario.Vehicle 
            % =============================================================
            arguments
                car {mustBeUnderlyingType(car,'driving.scenario.Vehicle')}       
            end
            states.Pose = [car.Position(1:2),...
                    car.Velocity(1:2),...
                    car.Yaw,...
                    0]';
            states.Length = car.Length;
            states.RearOverhang = car.RearOverhang;
            states.FrontOverhang =  car.Length-car.RearOverhang;
            states.Collaborating = 0;
        end % extract_vehicle_states_dsd

        
        
        
    end
    
    methods(Abstract, Access = protected)
        % Computation of the terminal conditions for each candidate pair
        [isFeasible, terminalTime, terminalPosition, disruption, ...
            cavCManType, trajectories, intersectTime] = ...
            compute_terminal_conditions(...
            obj, collabType, cav1Candidate, cav2Candidate, desSpeed)
    end
end
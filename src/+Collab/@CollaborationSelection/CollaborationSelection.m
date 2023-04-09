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
        % Total Energy [m/s^2]
        TotalEnergy (1,1) {mustBeNumeric} = inf;
        % CAV C ManeuverType
        CavCType (1,:) char {mustBeMember(CavCType,{'Selfish','SelfishRelaxed','Social','SelfishHDVs'})} = 'Selfish'
        % Maneuver Lenght [s]
        TerminalTime (1,1) {mustBeNumeric} = 0;
        % Terminal Positions Structure [m]
        TerminalPosition = struct('veh1',[],'veh2',[],'vehC',[]);
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
        % Plot Trajectories Estimation
        Display {mustBeNumericOrLogical} = false
        % Plot trajectory Results 
        DisplayResults {mustBeNumericOrLogical} = false
        % Create Results Log File
        ResultsLog {mustBeNumericOrLogical} = false
        % Results log file id
        ResultsFileID {mustBeNumeric} = -1
        % Create Estimations Log File
        EstimationLog {mustBeNumericOrLogical} = false  
        % Estimation log file id
        EstimationFileID {mustBeNumeric} = -1
        % Constant for time relaxation
        TimeRelaxationGain {mustBeNumeric} = 1.1
        % Min Speed for considering a CAV candidate [m/s]
        MinSpeed {mustBeNumeric} = 15 
        % Maximum allowed disruption [m^2]
        MaxDisruption {mustBeNumeric} = 0.200 
        % Minimum Delta safety distance [m]
        DeltaSafe {mustBeNumeric} = 4.3 
        % Reaction Time [s]
        ReactionTime = 0.6
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
        % Weights for Disruption Contribution [CAV1, CAV2, CAVC] Must Add
        % to 1
        Zeta = [0, 2/3, 1/3]
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

        function [isFeasible, maneuverEstimate] = estimate_trajectory(...
                    obj, cav1Candidate, cav2Candidate, desSpeed)
            % estimate_trajectory
            % -------------------------------------------------------------
            % Computes the trajectory for the candidate pair cav1Candidate
            % and cav2Candidate. The desired speed will be used to compute
            % the desired traffic flow speed
            % =============================================================
            % Check arguments
            arguments
                obj
                cav1Candidate
                cav2Candidate
                desSpeed = 33 %[m/s]
            end
            % Define the collaboration type
            candTypeCase = obj.define_collaboration_type(...
               cav1Candidate, cav2Candidate);
            % Compute candidate pair's terminal condition
            [isFeasible, terminalTime, terminalPosition, disruption, ...
                cavCManType, trajectories, intersectTime] = ...
                obj.compute_terminal_conditions(...
                candTypeCase, cav1Candidate, cav2Candidate, desSpeed);
            % Create container structure
            maneuverEstimate = struct();
            % If maneuver is infeasible, skip
            if ~isFeasible
                return
            end
            % Populate Solution Structure
            maneuverEstimate.TerminalTime = terminalTime;
            maneuverEstimate.TerminalPos = terminalPosition;
            maneuverEstimate.CollabType = candTypeCase;
            % Populate maneuver utility value
            maneuverEstimate.Disruption = disruption;
 
            % Extract Trajectory Estimates
            % CAV C
            cavCTraj = trajectories.cavC;
            cavCTraj.ManeuverType = cavCManType;
            cavCTraj.CAV = obj.CavC;  
            maneuverEstimate.cavCSol = cavCTraj;
            
            % CAV 1
            if ~isempty(cav1Candidate)
                cav1Traj = trajectories.cav1;
            else
                % Create empty structure
                cav1Traj = obj.create_empty_solution();
            end
            % Populate candidate
            cav1Traj.CAV = cav1Candidate;
            % Populate solution
            maneuverEstimate.cav1Sol = cav1Traj;

            % CAV 2
            if ~isempty(cav2Candidate)
                cav2Traj = trajectories.cav2;
                % Populate candidate
                cav2Traj.CAV = cav2Candidate;
                % Populate intersection time
                cav2Traj.IntersectTime = intersectTime;
            else
                % Create empty structure
                cav2Traj = obj.create_empty_solution();
            end
            % Populate candidate
            cav2Traj.CAV = cav2Candidate;
            % Populate solution
            maneuverEstimate.cav2Sol = cav2Traj;
            
            % Add Energy Consumption
            % Add total Ennergy consumption
            maneuverEstimate.TotalEnergy = ...
                maneuverEstimate.cavCSol.Energy + ...
                maneuverEstimate.cav2Sol.Energy + ...
                maneuverEstimate.cav1Sol.Energy ;
            % Add Flow Speed
            maneuverEstimate.DesSpeed = desSpeed;
            % Populate Maneuver Estimates
            if obj.EstimationLog
                obj.populate_esimation_log(maneuverEstimate,...
                    obj.EstimationFileID)
            end
           
        end

        function populate_esimation_log(obj, maneuverEstimate, fileID)
            % populate_esimation_log
            % -------------------------------------------------------------
            % populates the maneuver estimate for feasible candidate pairs
            % =============================================================
            % Extract maneuver terminal conditions from estimate
            terminalTime = maneuverEstimate.TerminalTime;
            terminalPosition = maneuverEstimate.TerminalPos;
            % Extract vehicles from estimate
            cav1 = maneuverEstimate.cav1Sol.CAV;
            cav2 = maneuverEstimate.cav2Sol.CAV;
            cavC = maneuverEstimate.cavCSol.CAV;
            % CAV C
            fprintf(fileID,obj.create_vehicle_log_string...
                (cavC,terminalPosition.vehC,terminalTime, ...
                maneuverEstimate.cavCSol));
            % CAV 1
            fprintf(fileID,obj.create_vehicle_log_string...
                (cav1,terminalPosition.veh1,terminalTime, ...
                maneuverEstimate.cav1Sol));
            % CAV 2
            fprintf(fileID,obj.create_vehicle_log_string...
                (cav2,terminalPosition.veh2,terminalTime, ...
                maneuverEstimate.cav2Sol));            
            % Maneuver
            fprintf(fileID, obj.create_maneuver_log_string(...
                maneuverEstimate, maneuverEstimate.cavCSol.ManeuverType));

        end % populate_log_maneuver

%         function [xf, traj, isFeasible, disruption, intersectionTime] = ...
%                 check_trajectory_intersection...
%             (obj, trajC, traj1, hdv2, terminalTime, desSpeed)
%             % check_trajectory_intersection
%             % -------------------------------------------------------------
%             % Given a time vector, trajectory of CAV1, trajectory of CAV C,
%             % and HDV2 initial conditions, computes a traejctory for which 
%             % HDV2 applies car folowing model with respect to CAV 1 and 
%             % then CAV C
%             % =============================================================
%             % Extract vehicle pose
%             hdv2Pose = obj.extract_vehicle_states(hdv2);
%             % Define dynamic model
%             f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
%             % Extract time Vector from Trajectory
%             timeVec = trajC.Time;
%             % number of iterations
%             N = length(timeVec);
%             % Integrate using RK4
%             dt = timeVec(end)/N;
%             % State Vector HDV2
%             X = zeros(2,N);
%             U = zeros(1,N);
%             % Interserction time vector buffer
%             intersectionTime = [];
%             % Initial conditions
%             X(:,1) = [hdv2Pose.Pose(1), hdv2Pose.Pose(3)];
%             for k=2:N % loop over control intervals
%                 % Check if CAV 1 or CAV C should be leader
%                 stateC = [trajC.Pos(k-1), trajC.Speed(k-1), trajC.Accel(k-1)];
%                 state1 = [traj1.Pos(k-1), traj1.Speed(k-1), traj1.Accel(k-1)];
%                 if stateC(3)>=0 && stateC(1)> X(1,k-1)+4
%                     % Compute acceleration under CAV C leader
%                     U(k) = obj.compute_commanded_acc(...
%                         X(:,k-1), stateC, obj.DeltaSafe);
%                     % Add intersection time
%                     intersectionTime = cat(1, intersectionTime, timeVec(k));
%                 else
%                     % Compute acceleration under CAV 1 leader
%                     U(k) = obj.compute_commanded_acc(...
%                         X(:,k-1), state1, obj.DeltaSafe);
%                 end
%                 % Runge-Kutta 4 integration
%                 X(:,k) = runge_kutta(f, dt, X(:, k-1), U(k));
%             end
%             % Define trajectory for HDV2 
%             traj.Pos = X(1,:);
%             traj.Speed = X(2,:);
%             traj.Accel = U(:);
%             traj.Energy = sum(0.5*dt*U(:).^2);
%             function X_next = runge_kutta(f, dt, X, U)
%                 k1 = f(X, U);
%                 k2 = f(X, U);
%                 k3 = f(X+dt/2*k2, U);
%                 k4 = f(X+dt*k3,   U);
%                 X_next = X + dt/6*(k1+2*k2+2*k3+k4);
%             end
% 
%             % Compute terminal position
%             xf = X(1,end);
%             % Determine feasibility
%             isFeasible = ~isempty(intersectionTime);
%             % compute disruption
%             disruption = obj.estimate_disruption(...
%                     traj, terminalTime, 'cav2', desSpeed);
%         end % check_trajectory_intersection

        function [xf, traj, isFeasible, disruption, intersectionTime] = ...
                check_trajectory_intersection...
            (obj, trajC, traj1, hdv2, terminalTime, desSpeed)
            % check_trajectory_intersection
            % -------------------------------------------------------------
            % Given a time vector, trajectory of CAV1, trajectory of CAV C,
            % and HDV2 initial conditions, computes a traejctory for which 
            % HDV2 applies car folowing model with respect to CAV 1 and 
            % then CAV C
            % =============================================================
            % Extract vehicle pose
            hdv2Pose = obj.extract_vehicle_states(hdv2);
            % Define dynamic model
            f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
            % Extract time Vector from Trajectory
            timeVec = trajC.Time;
            % number of iterations
            N = length(timeVec);
            % Integrate using RK4
            dt = timeVec(end)/N;
            % State Vector HDV2
            X = zeros(2,N);
            X1 = zeros(2,N);
            X2 = zeros(2,N);
            U = zeros(1,N);
            U1 = zeros(1,N);
            U2 = zeros(1,N);
            % Interserction time vector buffer
            intersectionTime = [];
            % Initial conditions
            X1(:,1) = [hdv2Pose.Pose(1), hdv2Pose.Pose(3)];
            X2(:,1) = [hdv2Pose.Pose(1), hdv2Pose.Pose(3)];
            for k=2:N % loop over control intervals
                % Check if CAV 1 or CAV C should be leader
                stateC = [trajC.Pos(k-1), trajC.Speed(k-1), trajC.Accel(k-1)];
                state1 = [traj1.Pos(k-1), traj1.Speed(k-1), traj1.Accel(k-1)];
                if stateC(3)>=0 && stateC(1)> X1(1,k-1)+10
                    % Compute acceleration under CAV C leader
                    U1(k) = obj.compute_commanded_acc(...
                        X1(:,k-1), stateC, obj.DeltaSafe);
                    % Add intersection time
                    intersectionTime = cat(1, intersectionTime, timeVec(k));
                else
                    % Compute acceleration under CAV 1 leader
                    U1(k) = obj.compute_commanded_acc(...
                        X1(:,k-1), state1, obj.DeltaSafe);
                end
                % Runge-Kutta 4 integration
                X1(:,k) = runge_kutta(f, dt, X1(:, k-1), U1(k));
            end
            for k=2:N % loop over control intervals
                % if CAV 1 should be leader               
                state1 = [traj1.Pos(k-1), traj1.Speed(k-1), traj1.Accel(k-1)];
                    % Compute acceleration under CAV 1 leader
                U2(k) = obj.compute_commanded_acc(...
                        X2(:,k-1), state1, obj.DeltaSafe);               
                % Runge-Kutta 4 integration
                X2(:,k) = runge_kutta(f, dt, X2(:, k-1), U(k));
            end
            Py = 1; %min(1,(trajC.Pos(1)-hdv2Pose.Pose(1))/(desSpeed*terminalTime+0.5*obj.Constraints.u_max*terminalTime^2+obj.DeltaSafe));
            Pn = 1-Py;
            X(1,:) = Py*X1(1,:) + Pn*X2(1,:);
            X(2,:) = Py*X1(2,:) + Pn*X2(2,:);
            U(:) = Py*U1(:)+Pn*U2(:);
            % Define trajectory for HDV2 
            traj.Pos = X(1,:);
            traj.Speed = X(2,:);
            traj.Accel = U(:);
            traj.Energy = sum(0.5*dt*U(:).^2);
            function X_next = runge_kutta(f, dt, X, U)
                k1 = f(X, U);
                k2 = f(X, U);
                k3 = f(X+dt/2*k2, U);
                k4 = f(X+dt*k3,   U);
                X_next = X + dt/6*(k1+2*k2+2*k3+k4);
            end

            % Compute terminal position
            xf = X(1,end);
            % Determine feasibility
            isFeasible = ~isempty(intersectionTime) && trajC.Pos(end)-xf>=obj.DeltaSafe;
            % compute disruption
            disruption = obj.estimate_disruption(...
                    traj, terminalTime, 'cav2', desSpeed);
        end % check_trajectory_intersection

        function [isfeasible, traj1, traj2, xf_1, xf_2] = check_cavC_safety(obj, trajC, cav1Candidate, cav2Candidate, terminalTime)
            %Extract vehicle pose
            %hdv1States = obj.extract_vehicle_states(cav1Candidate);
%             [xf_1, traj1, ~, ~] = obj.compute_longitudinal_trajectory_candidates (cav1Candidate,...
%                 'cav1', xf_C, vf_C, terminalTime, desSpeed);
            hdv1States = obj.extract_vehicle_states(cav1Candidate);
            hdv2States = obj.extract_vehicle_states(cav2Candidate);
            %Define dynamic model
            f = @(x,u) [x(2);u]; % dx/dt = f(x,u)
            % Extract time Vector from Trajectory
            timeVec = trajC.Time;
            % number of iterations
            N = length(timeVec);
            % Integrate using RK4
            dt = timeVec(end)/N;
            % State Vector HDV2
            X = zeros(2,N);
            X1 = zeros(2,N);
            X2 = zeros(2,N);
            U = zeros(1,N);
            U1 = zeros(1,N);
            U2 = zeros(1,N);
            trajectory1 = zeros(2,N);
            % Interserction time vector buffer
            intersectionTime = [];
            % Initial conditions
            X1(:,1) = [hdv2States.Pose(1), hdv2States.Pose(3)];
            X2(:,1) = [hdv2States.Pose(1), hdv2States.Pose(3)];
            x_1_0 = hdv1States.Pose(1);
            v_1_0 = hdv1States.Pose(3);
            for k=1:N
                x_1 = x_1_0 + v_1_0*dt*k;
                trajectory1(1,k)=x_1;
                trajectory1(2,k)=v_1_0;
                if k>=2 % loop over control intervals
                    % Check if CAV 1 or CAV C should be leader
                    stateC = [trajC.Pos(k-1), trajC.Speed(k-1), trajC.Accel(k-1)];
                    state1 = [trajectory1(1,k-1), trajectory1(2,k-1), 0];
                    if stateC(3)>=0 && stateC(1)> X1(1,k-1)+10
                        % Compute acceleration under CAV C leader
                        U1(k) = obj.compute_commanded_acc(...
                            X1(:,k-1), stateC, obj.DeltaSafe);
                        % Add intersection time
                        intersectionTime = cat(1, intersectionTime, timeVec(k));
                    else
                        % Compute acceleration under CAV 1 leader
                        U1(k) = obj.compute_commanded_acc(...
                            X1(:,k-1), state1, obj.DeltaSafe);
                    end
                    % Runge-Kutta 4 integration
                    X1(:,k) = runge_kutta(f, dt, X1(:, k-1), U1(k));
                end
            end
            for k=2:N % loop over control intervals
                % if CAV 1 should be leader               
                state1 = [trajectory1(1,k-1), trajectory1(2,k-1), 0];
                    % Compute acceleration under CAV 1 leader
                U2(k) = obj.compute_commanded_acc(...
                        X2(:,k-1), state1, obj.DeltaSafe);               
                % Runge-Kutta 4 integration
                X2(:,k) = runge_kutta(f, dt, X2(:, k-1), U(k));
            end
           % Py = min(1,(trajC.Pos(1)-hdv2Pose.Pose(1))/(desSpeed*terminalTime+0.5*obj.Constraints.u_max*terminalTime^2+obj.DeltaSafe));
            Py = 1;
            Pn = 1-Py;
            X(1,:) = Py*X1(1,:) + Pn*X2(1,:);
            X(2,:) = Py*X1(2,:) + Pn*X2(2,:);
            U(:) = Py*U1(:)+Pn*U2(:);
            % Define trajectory for HDV1
            traj1.Pos = trajectory1(1,:);
            traj1.Speed = trajectory1(2,:);
            traj1.Accel = zeros(1,N);
            traj1.Energy = 0;
            xf_1 = traj1.Pos(N);
            % Define trajectory for HDV2 
            traj2.Pos = X(1,:);
            traj2.Speed = X(2,:);
            traj2.Accel = U(:);
            traj2.Energy = sum(0.5*dt*U(:).^2);
            xf_2 = traj2.Pos(N);
            if trajC.Pos(N) - traj2.Pos(N) >= obj.DeltaSafe && trajectory1(1,N) - trajC.Pos(N) >= obj.DeltaSafe
                isfeasible = 1;
            else
                isfeasible = 0;
            end
            function X_next = runge_kutta(f, dt, X, U)
                k1 = f(X, U);
                k2 = f(X, U);
                k3 = f(X+dt/2*k2, U);
                k4 = f(X+dt*k3,   U);
                X_next = X + dt/6*(k1+2*k2+2*k3+k4);
            end
        end

        function disruption = estimate_disruption(obj,trajectory,...
                terminalTime, cavType, flowSpeed)
            % estimate_disruption
            % -------------------------------------------------------------
            % Defines disruption contribution given a trajectory history 
            % for CAV. The disruption is based on a weighted  sum between
            % terminal position difference and terminal speed
            % =============================================================
            % Extract terminal and initial conditions
                xf = trajectory.Pos(end);
                x0 = trajectory.Pos(1);
                v0 = trajectory.Speed(1);

            % Terminal speed contribution
            speed_disruption = (trajectory.Speed(end)-flowSpeed)^2/...
                max([obj.Constraints.v_max-flowSpeed, ...
                obj.Constraints.v_min-flowSpeed].^2);
            % Compute worst case scenario terminal position
            if obj.Constraints.u_min*terminalTime+v0>=obj.Constraints.v_min
                %Vehicle i exceeds x_f under the minimal acceleration
                x_f_worst = x0 + v0*terminalTime + ...
                    0.5*obj.Constraints.u_min*terminalTime^2;
            else % u_min*t_f+v_0<v_min
                %Vehicle exceeds xif under the minimal acceleration
                % after attaining its minimal velocity
                x_f_worst =   x0 + ...
                    (obj.Constraints.v_min^2-v0^2)/(2*obj.Constraints.u_min) + ...
                    obj.Constraints.v_min*(terminalTime-...
                    (obj.Constraints.v_min-v0)/obj.Constraints.u_min);
            end           
            % Terminal Position Contribution
            
            % Compute ideal terminal position
            x_f_ideal = x0+v0*terminalTime;
            % Compute maximum disruption
            maxDisruption = (x_f_ideal -  x_f_worst)^2;
            % Compute real disruption
            realDisruption = (xf -  x_f_ideal)^2;
            % Compute normalized disruption
            delta_disruption = realDisruption/maxDisruption;
            
            % Total disruption 
%             if strcmp(cavType,'cav1')
%                 disruption = 0;
%             else
                disruption = obj.AlphaDeltaX*speed_disruption + ...
                    obj.AlphaTerminalSpeed*delta_disruption;
%             end
        end % estimate_disruption

        function [isFeasible,terminalTime, terminalPosition, disruption,cavCManType,...
                 intersecTime, trajectories] = extract_solution(obj, solArray)
            % extract_solution
            % -------------------------------------------------------------
            % Extract maneuver estimate solution from structure
            % =============================================================

            % Check is solArray is empty
            if isempty(solArray)
                isFeasible = false;
                cavCManType = 'none';
                [terminalTime, terminalPosition, disruption, ...
                trajectories, intersecTime] = obj.create_dummy_output;
                return
            else
                % Extract indexes for minimum disruption
                [minDisruption, minIndex] = ...
                    min([solArray.Disruption]);
                if minDisruption <= obj.MaxDisruption
                    isFeasible = true;
                    optSol = solArray(minIndex);
                else
                    isFeasible = false;
                    cavCManType = 'none';
                    [terminalTime, terminalPosition, disruption, ...
                    trajectories, intersecTime] = obj.create_dummy_output;
                    return
                end
            end
            terminalTime = optSol.terminalTime ;
            terminalPosition = optSol.terminalPosition;
            disruption = optSol.Disruption;
            cavCManType = optSol.cavCManType;
            intersecTime = optSol.intersecTime;
            trajectories = optSol.trajectories;
        end %extract_solution

        function formatedString = create_vehicle_log_string...
                (obj, cav,terminalPosition,terminalTime, trajectory)
            % create_vehicle_log_string
            % Creates a formatted string containing the relevant values for
            % the cav passed as an argument
            % =============================================================
            % Check if cav is empty
            if ~isempty(cav)
                if strcmp(cav.RollType,'none')
                    id = 'none';
                    vehicleType = 'virtual';
                    x_0 = nan;      
                    v_0 = nan;     
                    x_f = nan;
                    v_f = nan;
                    delta = nan;
                else
                    candVehPose = obj.extract_vehicle_states(cav);
                    id = cav.VehicleID;   
                    vehicleType = cav.VehicleType;
                    x_0 = candVehPose.Pose(1);      
                    v_0 = hypot(candVehPose.Pose(3), candVehPose.Pose(4));     
                    x_f = terminalPosition;
                    v_f = trajectory.Speed(end);
                    delta = (x_f-(x_0+v_0*terminalTime))^2;
                end
            else
                id = 'none';
                vehicleType = 'virtual';
                x_0 = nan;      
                v_0 = nan;     
                x_f = nan;
                v_f = nan;
                delta = nan;
            end
            % Create formated string
            formatedString = sprintf(...
                '%s \t %s \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t ',...
                id,vehicleType, x_0, v_0, x_f,v_f, delta);
        end % create_vehicle_log_string
        
    end

    methods(Static, Access = protected)
        function cavCandidatesSol= appendSol(...
                isFeasible, maneuverEstimate, cavCandidatesSol)
            % -------------------------------------------------------------
            % Appends trajectory solutions for every feasible candidate
            % pair onto a candidatesSol array
            % =============================================================
            % if not feasible, skip
            if isFeasible
                % Store solution
                if isempty(cavCandidatesSol)
                    cavCandidatesSol = maneuverEstimate;
                else
                    cavCandidatesSol= cat(2,cavCandidatesSol,maneuverEstimate);
                end
            end
        end
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

        function emptyStruct = create_empty_solution()
            % createEmptySolution
            % -------------------------------------------------------------
            % Creates an empty structure populating solution field with
            % mock up values
            % =============================================================
            emptyStruct.Pos = 0;
            emptyStruct.Speed = 0;
            emptyStruct.Accel = 0;
            emptyStruct.Energy = 0;
            emptyStruct.CAV = nan;
        end %create_empty_solution

        function typeCase = define_collaboration_type(cav1Candidate, cav2Candidate)
            % choose_maneuver_type
            % -------------------------------------------------------------
            % Computes maneuver type based on available collaboration
            % vehicles
            % =============================================================
            typeCase = [];
            % Check if CAV1 and CAV2 exist            
            if ~isempty(cav1Candidate)&&~isempty(cav2Candidate)
                % Case 1 and 2: {CAV1, CAV2, CAVC}
                if strcmp(cav1Candidate.VehicleType,'CAV') && ...
                        strcmp(cav2Candidate.VehicleType,'CAV')
                    % Case1: {CAV1, CAV2, CAVC}, CAV1 is unconstrained in front
                    if isempty(cav1Candidate.Leader)
                        typeCase = 1;
                    % Case2: {CAV1, CAV2, CAVC}, CAV1 is constrained in front
                    else              
                        typeCase = 2;
                    end                    
                % Case 3 and 4: {CAV1, CAVC}
                elseif strcmp(cav1Candidate.VehicleType,'CAV') && ...
                        ~strcmp(cav2Candidate.VehicleType,'CAV')
                    % Case3: {CAV1, CAVC}, CAV1 is unconstrained in front
                    % Check if cav1 is leader (vehicle is unconstrained)
                    if isempty(cav1Candidate.Leader)    
                        typeCase = 3;
                    % Case4: {CAV1, CAVC}, CAV1 is constrained in front
                    else
                        typeCase = 4;
                    end
                % Case5: {CAV2, CAVC}, CAV2 is unconstrained in the front
                elseif ~strcmp(cav1Candidate.VehicleType,'CAV') && ...
                        strcmp(cav2Candidate.VehicleType,'CAV')
                    % Set case number
                    typeCase = 5;
                % Case6: {HDV1, HDV2, CAVC}, 
                elseif ~strcmp(cav1Candidate.VehicleType,'CAV') && ...
                        ~strcmp(cav2Candidate.VehicleType,'CAV')
                    % Set case number
                    typeCase = 4;
                end
            % Case34: {CAV1, CAVC}, CAV1 is unconstrained in front
            elseif ~isempty(cav1Candidate) && isempty(cav2Candidate)
                % Case3: {CAV1, CAVC}, CAV1 is unconstrained in front
                % Check if cav1 is leader (vehicle is unconstrained)
                if isempty(cav1Candidate.Leader)
                    typeCase = 3;
                % Case4: {CAV1, CAVC}, CAV1 is constrained in front
                else
                    typeCase = 4;
                end                
            % Case5: {CAV2, CAVC}, CAV2 is unconstrained in the front
            elseif isempty(cav1Candidate) && ~ isempty(cav2Candidate)
                % Set case number
                typeCase = 5;
            end
        end % define_collaboration_type

        function acc = compute_commanded_acc(...
                egoStates, leaderStates, delta)
            % compute_commanded_acc
            % -------------------------------------------------------------
            % Computes the commanded acceleration given a car following 
            % model (Gipps) and a vector of states for hdvStates and 
            % leaderStates [pos, speed, accel][m, m/s, m/s^2] and a delta 
            % value for spacing betweeb vehicles
            % =============================================================
            speedDiff = leaderStates(2) - egoStates(2);
            currSpeed = egoStates(2);
            leaderSpacing = leaderStates(1) -  delta;
            % Compute acceleration using IDM
            acc =  DrivingModel.gippsDriverModel(...
                leaderSpacing,currSpeed,speedDiff);
        end %compute_commanded_acc

        function formatedString = create_maneuver_log_string(...
                maneuverEstimate, CavCManeuverType)
            % create_maneuver_log_string
            % -------------------------------------------------------------
            % creates a formatted string containing maneuver details for a
            % collaboration triplet
            % =============================================================
            % Extract maneuver terminal conditions from estimate
            terminalTime = maneuverEstimate.TerminalTime;
            disruption = maneuverEstimate.Disruption;
            energy = maneuverEstimate.TotalEnergy;
            desSpeed = maneuverEstimate.DesSpeed;
            % Create formated String
            formatedString = sprintf(...
                '%.2f \t %.2f \t %.2f \t %.2f \t %s \n',...
                terminalTime, disruption, energy,...
                desSpeed, CavCManeuverType);
        end

        function isFeasible = check_safety_distance(...
                obstaclePose, tf, xf, deltaX, type)
            % check_safety_distance
            % -------------------------------------------------------------
            % checks if ego vehicle will collide at terminal position xf
            % under assumption that obstacle vehicle travels at constant
            % speed
            % =============================================================
            x_o_0 = obstaclePose(1);
            v_o_0 = sqrt(sum(obstaclePose(3:4).^2));
            
            switch type
                case 'front'
                    isFeasible = (xf<=x_o_0+v_o_0*tf-deltaX);
                case 'back'
                    isFeasible = (xf>=x_o_0+v_o_0*tf+deltaX);
            end
        end %check_safety_distance
        
        function [terminalTime, terminalPosition, disruption, ...
            trajectories, intersectTime] = create_dummy_output()
            % create_dummy_output
            % -------------------------------------------------------------
            % Creates an empty maneuver solution for infeasible maneuvers
            % =============================================================
            terminalTime = 0;
            terminalPosition.veh1 = 0;
            terminalPosition.veh2 = 0;
            terminalPosition.vehC = 0;
            disruption = inf;
            intersectTime = inf;
            %individual Trajectory
            traject = struct('Time',[],'Pos', [], 'Speed', [], ...
                'Accel', [],'Energy', 0);
            trajectories.cav1 = traject;
            trajectories.cav2 = traject;
            trajectories.cavC = traject;
        end % create_dummy_output

        function solStruct = populate_solution_structure(...
                terminalTime, xf_1, xf_2, xf_C, disruption,cavCManType,...
                iteration, intersecTime, traj1, traj2, trajC)
            % populate_solution_structure
            % -------------------------------------------------------------
            % populates a structure with solution parameters for a maneuver
            % estimate given a candidate pair
            % =============================================================
            solStruct.terminalTime = terminalTime;
            solStruct.terminalPosition.veh1 = xf_1;
            solStruct.terminalPosition.veh2 = xf_2;
            solStruct.terminalPosition.vehC = xf_C;
            solStruct.Disruption = disruption;
            solStruct.cavCManType = cavCManType;
            solStruct.iteration = iteration;
            solStruct.intersecTime = intersecTime;
            solStruct.trajectories.cav1 = traj1;
            solStruct.trajectories.cav2 = traj2;
            solStruct.trajectories.cavC = trajC;
        end %populate_solution_structure

         function solStruct = create_dummy_solution_structure(obj)
            % create_dummy_solution_structure
            % -------------------------------------------------------------
            % populates a structure with solution parameters for a maneuver
            % estimate given a candidate pair
            % =============================================================
            [terminalTime, terminalPosition, disruption, ...
                    trajectories, intersectTime] = obj.create_dummy_output;
            solStruct.terminalTime = terminalTime;
            solStruct.terminalPosition = terminalPosition;
            solStruct.Disruption = disruption;
            solStruct.cavCManType = 'none';
            solStruct.iteration = 0;
            solStruct.intersecTime = intersectTime;
            solStruct.trajectories = trajectories;
        end %create_dummy_solution_structure

        function [xf, trajectory] = extract_terminal_conditions(...
            posHist, speedHist, accelHist, energy, terminalTime)
            % extract_terminal_conditions
            % -------------------------------------------------------------
            % Extracts the terminal position from trajectory. Additionally,
            % it creates a date structure containing the trajectory history
            % for vehicle
            % =============================================================
            xf = posHist(end);
            trajectory.Time = linspace(0,terminalTime,length(posHist));
            trajectory.Pos = posHist;
            trajectory.Speed = speedHist;
            trajectory.Accel = accelHist;
            trajectory.Energy = energy;
        end %extract_terminal_conditions

    end
    
    methods(Abstract, Access = protected)
        % Computation of the terminal conditions for each candidate pair
        [isFeasible, terminalTime, terminalPosition, disruption, ...
            cavCManType, trajectories, intersectTime] = ...
            compute_terminal_conditions(...
            obj, collabType, cav1Candidate, cav2Candidate, desSpeed)
    end
end
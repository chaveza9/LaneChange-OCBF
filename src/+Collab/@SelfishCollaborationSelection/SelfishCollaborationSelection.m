classdef SelfishCollaborationSelection < Collab.CollaborationSelection
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    methods
        function obj = SelfishCollaborationSelection(cavC, vehicleU, ...
                cavCandidates, varargin)
            % Initialize parent class
            obj = obj@Collab.CollaborationSelection(cavC, vehicleU, cavCandidates);
            if nargin > 3
                setProperties(obj,nargin-3,varargin{:});
            end
        end
    end

    methods(Access=protected)
        function [isFeasible, terminalTime, terminalPosition, disruption, ...
            cavCManType, trajectories, intersectTime] = ...
            compute_terminal_conditions(...
            obj, collabType, cav1Candidate, cav2Candidate, desSpeed)
            % compute_terminal_conditions
            % -------------------------------------------------------------
            % Computes the terminal conditions for the candidate pair.
            % Additionally, it computes the numerical solution of the
            % trajectories that are performed.
            % =============================================================
            %% Create empty Structure buffer            
            [terminalTime, terminalPosition, disruption, ...
                    trajectories, intersectTime] = obj.create_dummy_output;
            %% Initialize rolls for every vehicle in the triplet
            % CAV C
            % Define initial Cav C maneuver roll
            cavCManType = 'Selfish';
            % Define CAV C's roll
            hasDefinedRoll = obj.CavC.define_cav_roll(cavCManType, ...
                'cavC', desSpeed, true);
            % CAV 1
            if ~isempty(cav1Candidate) && any(collabType == [1,2,3,4]) 
                if strcmp(cav1Candidate.VehicleType,'CAV')
                    hasDefinedRoll = hasDefinedRoll && ...
                        cav1Candidate.define_cav_roll('Accel', ...
                    'cav1', desSpeed, true);
                end
            end
            % CAV 2
            if ~isempty(cav2Candidate) && any(collabType == [1,2,5]) 
                if strcmp(cav2Candidate.VehicleType,'CAV')
                    hasDefinedRoll = hasDefinedRoll && ...
                        cav2Candidate.define_cav_roll('Decel', ...
                    'cav2', desSpeed, true);
                end
            end
            % HDV1 & HDV2
            

            % Return if not able to define rolls
            if ~hasDefinedRoll
                % Return infeasible maneuver
                isFeasible = hasDefinedRoll;
                return
            end

            
            %% Estimate Optimal Trajectory
            % Extract options
            % Compute numerical solution for minimum time CAV C trajetory 
            [isFeasibleC, xf_C, vf_C, trajC, terminalTime] = ...
                obj.compute_longitudinal_trajectory_ego (0,cav1Candidate,cav2Candidate);
            if ~isempty(cav1Candidate) && ~isempty(cav2Candidate) %&& collabType == 4
                [feasibleC,~,~,~,~] = obj.check_cavC_safety(trajC, cav1Candidate, cav2Candidate, terminalTime);
                isFeasibleC = isFeasibleC &  feasibleC;
                
            end
            % Return if CAV C's trajectory is infeasible
            if ~isFeasibleC
                isFeasible = isFeasibleC;
                return
            end            
            % helper variable
            solArray = [];
            %% Compute iteration 0 under no relaxation
            [isFeasible, solStruct] = ...
                    obj.compute_estimation_iteration...
                    (cav1Candidate, cav2Candidate, xf_C, vf_C, trajC, ...
                    terminalTime, desSpeed, cavCManType, 1);
            % Concatenate solution if feasible
            if isFeasible
                solArray = solStruct;
            end
            %% Compute solution under time Relaxation
            % ReDefine CAV C Roll just once                
            cavCManType = 'SelfishRelaxed';
            % Define CAV C's roll
            obj.CavC.define_cav_roll(cavCManType,'cavC', desSpeed, true);
            % Iterate Through Solutions
            for i=2:obj.relaxationTimes
            %parfor(i = 2:20, obj.Workers)
                % Relax time
                tf = terminalTime*(obj.TimeRelaxationGain)^i; 
                % Compute trajectory CAV C under relaxed conditions
                [isFeasibleC, xf_C, vf_C, trajC, tf] = ...
                obj.compute_longitudinal_trajectory_ego (tf, cav1Candidate, cav2Candidate);
                % compute trajectory for cav 1 and cav 2
                if isFeasibleC
                    [isFeasible, solStruct]  = ...
                        obj.compute_estimation_iteration...
                        (cav1Candidate, cav2Candidate, xf_C, vf_C, trajC, ...
                        tf, desSpeed, cavCManType, i);
                    if isFeasible
                        solArray = cat(1,solArray,solStruct);
                    end
                end

            end
            
            % Clear out parallel workers
            % delete(gcp("nocreate"));
            % Extract optimal solution 
            [isFeasible,terminalTime, terminalPosition, disruption,...
                cavCManType, intersectTime, trajectories] = ...
                 obj.extract_solution(solArray);         
        end % compute_terminal_conditions

        function [isFeasible, solStruct] = compute_estimation_iteration...
                (obj,cav1, cav2, xf_C, vf_C, trajC, ...
                terminalTime, desSpeed, cavCManType, iteration)
            % compute_estimation_iteration
            % -------------------------------------------------------------
            % Computes single estimation iteration for cav 1 and cav 2
            % =============================================================
            % Create placeholder
            intersecTime = [];
            %% Disruption C
            disruptionC = obj.estimate_disruption(...
                    trajC, terminalTime, 'cavC', desSpeed);
            %% Compute trajectory CAV 1  
            if strcmp(cav1.VehicleType,'CAV')
            [xf_1, traj1, isFeasible1, disruption1] = ...
                obj.compute_longitudinal_trajectory_candidates (cav1,...
                'cav1', xf_C, vf_C, terminalTime, desSpeed);
            else
                [isFeasible1, traj1,~, xf_1, ~] = ...
                    obj.check_cavC_safety(trajC,cav1,cav2,terminalTime);
                disruption1 = obj.estimate_disruption(...
                    traj1, terminalTime, 'cav1', desSpeed);
            end
                        
            
            %% Compute trajectory CAV 2 
%             if strcmp(cav2.VehicleType,'CAV')
%                 [xf_2, traj2, isFeasible2, disruption2] = ...
%                 obj.compute_longitudinal_trajectory_candidates (cav2,...
%                 'cav2', xf_C, vf_C, terminalTime, desSpeed);
%             %% Compute HDV2 intersection times if HDV2 exists
%             else
%                 % Check if trajectories between CAV 2 and CAV C
%                 % intersect
%                 [xf_2, traj2, isFeasible2, disruption2, intersecTime] = ...
%                 obj.check_trajectory_intersection...
%                     (trajC, traj1, cav2, terminalTime, desSpeed);
%             end
 %           if strcmp(cav2.VehicleType,'CAV')
            [xf_2, traj2, isFeasible2, disruption2] = ...
            obj.compute_longitudinal_trajectory_candidates (cav2,...
            'cav2', xf_C, vf_C, terminalTime, desSpeed);
%             else
%                 [isFeasible2, ~, traj2, ~, xf_2] = ...
%                     obj.check_cavC_safety(trajC,cav1,cav2,terminalTime);
%                 disruption2 = obj.estimate_disruption(...
%                     traj2, terminalTime, 'cav2', desSpeed);
%             end

            %% Compute HDV2 intersection times if HDV2 exists
            if ~isempty(cav2)
                if ~strcmp(cav2.VehicleType,'CAV') && ~isFeasible2
                    % Check if CAV C  can perform maneuver trhough 
                    % collaborations from HDV2 by Checking if trajectories 
                    % between CAV 2 and CAV C intersect
                    [xf_2, traj2, isFeasible2, disruption2, intersecTime] = ...
                    obj.check_trajectory_intersection...
                        (trajC, traj1, cav2, terminalTime, desSpeed);
                end
            end
            
            % Determine Feasibility of Candidate Pair
            isFeasible = all([isFeasible1,isFeasible2]);
            % Add disruption values 
            disruption = obj.Zeta(1)*disruption1 + ...
                obj.Zeta(2)*disruption2 + ...
                obj.Zeta(3)*disruptionC;
            % Populate solution structure
            if isFeasible
                solStruct = ...
                    obj.populate_solution_structure(...
                    terminalTime, xf_1, xf_2, xf_C, ...
                    disruption,cavCManType,...
                    iteration, intersecTime, traj1, traj2, trajC);
            else
                solStruct = obj.create_dummy_output;
            end

        end % compute_estimation_iteration
        
        function [xf, trajectory, isFeasible, disruption] = ...
                compute_longitudinal_trajectory_candidates (obj, cav, cavType, ...
                xf_C, vf_C, terminalTime, desSpeed)
            % compute_longitudinal_trajectory_candidates
            % -------------------------------------------------------------
            % Computes longituinal trajectory for cav1 or cav2
            % =============================================================
            % Generate empty trajectory buffer
            trajectory = struct('Time',[],'Pos', [], 'Speed', [], ...
                'Accel', [],'Energy', inf);
            % Generate terminal position
            xf = 0;
            % Generate disruption placeholder
            disruption = 0;
            % Generate feasibility placeholder
            isFeasible = 1;
            % Check if cav is empty
            if isempty(cav)
                return
            end
            % Compute solution
            if strcmp(cav.VehicleType,'CAV')
                [isFeasible, ~, posHist, speedHist, ...
                    accelHist, energy] =...
                    cav.compute_longtudinal_trajectory (...
                    terminalTime, xf_C, nan, ...
                    'display', obj.Display,'estimate',true); 
                % Populate structure with cav 
                [xf, trajectory] = obj.extract_terminal_conditions(...
                    posHist, speedHist, accelHist, energy, terminalTime);
                disruption = obj.estimate_disruption(...
                    trajectory, terminalTime, cavType, desSpeed);
            elseif ~strcmp(cav.VehicleType,'CAV') && strcmp(cavType, 'cav1')
                % Extract vehicle pose
                vehiclePose = obj.extract_vehicle_states(cav);
                % compute minimum safety distance between c and 1
                deltaX_C1 = obj.DeltaSafe + vf_C*obj.ReactionTime;
                isFeasible = obj.check_safety_distance(...
                    vehiclePose.Pose, terminalTime, xf_C, deltaX_C1,'front');
%                 [xf, trajectory] = cav.compute_longtudinal_trajectory(...
%                     terminalTime, xf_C, nan, ...
%                     'display', obj.Display,'estimate',true);
%                 disruption = obj.estimate_disruption(...
%                     trajectory, terminalTime, cavType, desSpeed);

            elseif ~strcmp(cav.VehicleType,'CAV') && strcmp(cavType, 'cav2')
                % Extract vehicle pose
                vehiclePose = obj.extract_vehicle_states(cav);
                % compute minimum safety distance between c and 2
                deltaX_C2 = obj.DeltaSafe + desSpeed*obj.ReactionTime;
                isFeasible = obj.check_safety_distance(...
                    vehiclePose.Pose, terminalTime, xf_C, deltaX_C2,'back');

            end            
        end % compute_longitudinal_trajectory_candidates

        function [isFeasible, xf, vf, trajectory, terminalTime] = ...
                compute_longitudinal_trajectory_ego (obj,...
                terminalTime, cav1Candidate, cav2Candidate)
            % compute_longitudinal_trajectory_ego
            % -------------------------------------------------------------
            % Computes the ego longitudinal trajectory
            % =============================================================
            % Extract vehicle U states
            vehUStates = obj.extract_vehicle_states(obj.VehicleU);
            cav1States = obj.extract_vehicle_states(cav1Candidate);
            cav2States = obj.extract_vehicle_states(cav2Candidate);

            % Compute trajectory
            [isFeasible, terminalTime, posHist, speedHist, ...
                accelHist, energy] =...
                obj.CavC.compute_longtudinal_trajectory (...
                terminalTime, 0, vehUStates.Pose, ...
                'display', obj.Display, 'estimate', true, ...
                'includeCav1Cav2', true, 'cav1States', cav1States, ...
                'cav2States', cav2States);
            % Populate structure with cav C
            [xf, trajectory] = obj.extract_terminal_conditions(...
                posHist, speedHist, accelHist, energy, terminalTime);
            % Compute CAV C terminal Speed
            vf = speedHist(end);

        end % compute_longitudinal_trajectory_ego
    end

    methods (Static, Access=private)



        
        % Selfish Terminal Position computation
        [Xf_1, Xf_2, Xf_C, fval, flag] = ...
            optimization_terminal_position_selfish(veh1, veh2,...
            vehC, xcf, vehU_2, t_f, params)
    end
end
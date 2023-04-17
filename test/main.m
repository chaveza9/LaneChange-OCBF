%% Setup
clc;
clear all;  %#ok<CLALL> 
close all;
warning off;
%% Default
addpath('..\Solvers\casadi-windows-matlabR2016a-v3.5.5')
feature('COM_SafeArraySingleDim', 1); % Matlab should only pass one-dimensional array to COM
scenario = 'StraightHighwayCorridor_Vissim11_baseline'; % Scenario name
Note = '0d8DeltaX_0d2DeltaSpeedDifference_MaxDisruption0d15_dist10_includedNonCollabReq';
%Note = 'BaselineNoBlockedTypes';
%% Recording Options
VideoGeneration = 0; % Generates Videos of the Maneuver
Verbose = true;          % Prints Maneuver Results on the console
EstimationResults = 0; % Generates a file with all estimation values for feasible candidate pairs
FinalResults = 0;    % Generates a file with only the selected candidate pairs for every maneuver
PlotEstimation = 0;   % Generates plots for every maneuver
PlotResults = 0;   % Generates plots for every maneuver
visualization3D = 0;
%% Simulation Parameters
% Traffic Variables
Density = 2000;
CAVRelDistribution = 1; % Percentage of Cooperative CAVS  in Maneuver
% Global variables
MannedVehicleType = 201; % Vehicle type code for manned vehicles
CarType = 100;
CavType_noLaneChange = 105;
CavType = 1011; % Vehicle type CAV code
CavCType = 102;
Cav1Type = 103;
Cav2Type = 104;
CavCSocial = 1012;	%CAVC social
CavCSocialRel = 1013;	%CAVC social relaxed
CavCWait = 1014;	% CAVC wait
Uncooperative = 106;
cameraHeight = 50;
% Define Vissim Simulation Time parameters
SampleTime = 0.05;           % Simulation's time step size [sec]
StopTime = 120*2;               % Final simulation size [sec]
% Starting maneuver distance parameters
max_dist = 80;
min_dist = 60;
startRelDistance = 70;
% Vehicle constraints
constraints.u_max = 3.3;     % Vehicle i max acceleration
constraints.u_min = -7;      % Vehicle i min acceleration
constraints.v_min = 10;      % Vehicle i min velocity
constraints.v_max = 35;      % Vehicle i max velocity
constraints.safetyDist = 25; % Vehicle i safety distance
% Vehicle dimensions
global params %#ok<GVMIS> 
params.actors.carLen   = 4.7; % [m]
params.actors.carWidth = 1.8; % [m]
params.actors.rearAxleRatio = .25;
% Define road dimensions
params.road.laneWidth   = 3.6; % [m] 
params.road.length = 3000; %[m]
% CAV 1 parameter definition
% Distance relative to U to define CAV1 [m]
constraints.deltaCav1 = 35; 
constraints.deltaCavCRear = 80;
constraints.deltaCavUFront = 50;

% Iteration type
iterType = 'distance';

%% Initiate Simulation
% Open Vissim Server
Vissim = actxserver('Vissim.Vissim.11');
% Open file 
[currFldr,~,~] = fileparts(which(mfilename));
scenarioFldr = [currFldr filesep 'env' filesep scenario filesep];
flag_read_additionally  = false;
Vissim.LoadNet([scenarioFldr scenario '.inpx'],flag_read_additionally);
% Vissim.LoadLayout([scenarioFldr scenario '.layx']);
sim=Vissim.simulation;
sim.set('AttValue', 'SimRes', 1/SampleTime);
sim.set('AttValue','SimBreakAt', StopTime);
% Change visualization to 3D
set(Vissim.Graphics.CurrentNetworkWindow, 'AttValue', '3D', visualization3D);
%% Create Vehicle Composition
compositionKey = 4; % Desired Simulation Vehicle Composition Key 
% Extract vehicle composition relative flow
relFlow = Vissim.Net.VehicleCompositions.ItemByKey(compositionKey).VehCompRelFlows.GetAll;
cavRelFlow = relFlow{2};
uncooperativeRelFlow = relFlow{3};
% Set Vehicle Composition Relative Flows
cavRelFlow.set('AttValue','RelFlow',CAVRelDistribution);
uncooperativeRelFlow.set('AttValue','RelFlow',1-CAVRelDistribution+0.001);
%% Add Vehicle inputs
% CAV Traffic Demand
link1 = Vissim.Net.Links.ItemByKey(1); % Get link
cavInpput = Vissim.Net.VehicleInputs.AddVehicleInput(1,link1);
cavInpput.set('AttValue', 'Volume(1)', Density);
cavInpput.set('AttValue', 'VehComp(1)', compositionKey); % CAV distribution
% Truck setup
linkTruck = 1; % Network link
laneTruck = 1;
xCoordinateTruck = 100; %distance from start of link m
speedTruck = 60; % Truck speed km/h
sim.RunSingleStep;
vehicleUVisObj = Vissim.Net.Vehicles.AddVehicleAtLinkPosition(...
    MannedVehicleType, linkTruck, laneTruck, xCoordinateTruck, speedTruck);
veh_U_ID = get(vehicleUVisObj, 'AttValue', 'No');
vehU =  extract_vehicle_states_Vissim (Vissim,veh_U_ID);
%% Setup video folders
name = [Note,num2str(CAVRelDistribution*100),'_percent'];
logFilename = sprintf([name,'_Log%s'], datestr(now,'mm-dd-yyyy_HH-MM'));
TOD = datestr(now,'mm-dd-yyyy_HH-MM');
resultsFilename = sprintf([name,'_Results%s'], TOD);
% Create logging folders
if VideoGeneration 
    videoFldr = [currFldr, filesep, 'Results', filesep, logFilename];  %#ok<UNRCH>
    mkdir(videoFldr);
end
if EstimationResults || VideoGeneration || FinalResults
    ResultsFolder = [currFldr, filesep, 'Results', filesep, TOD];
    mkdir(ResultsFolder);
end
% Create logging file
if EstimationResults 
    EstimationlogFileID = fopen([ResultsFolder, filesep, logFilename, '.csv'],'w');   %#ok<UNRCH>
    create_log_headers(EstimationlogFileID);
else
    EstimationlogFileID = 0;   
end
if FinalResults
    FinalResultsFileID = fopen([ResultsFolder, filesep, resultsFilename, '.csv'],'w');  %#ok<UNRCH> 
    create_log_headers(FinalResultsFileID);
else
    FinalResultsFileID = 0; 
end
% Create Log File Descriptor
if FinalResults || EstimationResults 
    % Check if File exists
    if ~exist('.\Results\ResultsFileDescription.csv',"file")
        FileDescriptorID = fopen(".\Results\ResultsFileDescription.csv",'w');
        create_results_descriptor_headers(FileDescriptorID)
    % Otherwise Create a new Log File Descriptor 
    else
        FileDescriptorID = fopen(".\Results\ResultsFileDescription.csv",'a+');
    end
end
%% Add Comment description to vissim simulation run
simRuns = Vissim.Net.SimulationRuns.GetAll;
set(simRuns{end}, 'AttValue', 'Comment', Note);
%% Simulation
currTime = 0; % Start Time
timer = 0;
startTimer = false;
max_time = 1; % Seconds
% helper flags
startManeuver = false;
isManeuverRunning = false;
count = 0;
imgCount = 0;
set(0,'DefaultTextFontName','Times',...
'DefaultTextFontSize',18,...
'DefaultAxesFontName','Times',...
'DefaultAxesFontSize',18,...
'DefaultLineLineWidth',1,...
'DefaultLineMarkerSize',7.75)
while currTime <= StopTime    
    % Perform Simulation Step
    sim.RunSingleStep;
    % Update Current Time
    currTime = sim.SimulationSecond
    % Update Vehicle U States
    vehU =  extract_vehicle_states_Vissim (Vissim,veh_U_ID);
    % Update chase plot
    if visualization3D
        create_chase_visualization3d(Vissim, vehU, 90, cameraHeight); %#ok<UNRCH> 
    else
        create_chase_visualization2d(Vissim, vehU, 90, cameraHeight);
    end
    % Start Maneuver
    if ~isManeuverRunning && ~startTimer
        % Check for Followers
        attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane'};
        cav_C_Vissim = extract_neighbor(Vissim, vehU.No,'follower', attributes, 1);
        % Check if Follower should start maneuver
        if ~isempty(cav_C_Vissim) && ~startManeuver
            if vehU.Pos-cav_C_Vissim.Pos <= startRelDistance
                % Change flag to true
                startManeuver = true;
                startManeuver = false;
                % extract vehicle vissim object to control vehicle
                % CAVC
                [cavC, cavCVisObj] = ...
                    extract_vehicle_states_Vissim (Vissim,cav_C_Vissim.No);
                startRelDistance = min_dist+(max_dist-min_dist).*rand;
            end
        end    

        % If Request for collaboration is sent, start maneuver
        if startManeuver 
            % Create a driving scenario
            scenario = ds4vehicleScenario(params);
            % Look for CAV1 and CAV2 (if they exist)
            % CAVC
            if ~isempty(cavC)
                % Change vehicle color and type to CAVC
                set(cavCVisObj, 'AttValue', 'VehType', CavCType);
                % Initialize Vehicle Dynamics
                vehicleC = create_cav (scenario,cavC,'CAV', SampleTime, ...
                    StopTime, constraints);
                set(cavCVisObj, 'AttValue', 'ExtContr', 4);
            end
            % Vehicle U
            if ~isempty(vehU)
                % Initialize Vehicle Dynamics
                vehicleU = create_cav (scenario, vehU,'NonControlled', SampleTime, ...
                    StopTime, constraints);
            end
            % Identify cav candidates (neighbourhood vehicles)
            if strcmp(iterType,'number')
                [candVissim, candCAV] = find_cav_candidates_number...
                    (Vissim, vehU, 5, scenario,...
                    SampleTime, StopTime, constraints);
            else
                [candVissim, candCAV] = find_cav_candidates_distance...
                    (Vissim,cavC, vehU, scenario, SampleTime, ...
                    StopTime, constraints, Uncooperative);
            end

            % Create single maneuver class
            singleManeuverObj = SingleManeuver(vehicleC, vehicleU, candCAV,...
                scenario,  'Constraints', constraints, 'Verbose', Verbose, ...
                'Display', PlotEstimation,...
                'DisplayResults', PlotResults,...
                'EstimationLog', EstimationResults, ...
                'EstimationFileID', EstimationlogFileID,...
                'ResultsLog',FinalResults,'ResultsFileID',FinalResultsFileID);
            % Compute Optimal collaboration set
            [isManeuverRunning,  vehicle1, vehicle2, vehicleC, isCollabReq] = ...
                singleManeuverObj.find_optimal_pair;
            % Check if collaboration is not needed
            if ~isCollabReq
                % Command Lane Change
                link_number = 1;
                cavC = extract_vehicle_states_Vissim (Vissim,cav_C_Vissim.No);
                cavCVisObj.MoveToLinkPosition(link_number, 2, cavC.Pos);
                set(cavCVisObj, 'AttValue', 'VehType', CavType_noLaneChange);
                set(cavCVisObj, 'AttValue', 'DesSpeed', constraints.v_max*3.6); 
                % set flags back to false
                isManeuverRunning = false;
                startManeuver = false;
                count = count+1;
            end
            % if collaboration pair was found, start maneuver
            if isManeuverRunning
                isLongManRunning = singleManeuverObj.start_longitudinal_maneuver;
            end
            % Check if maneuver should be cancelled
            if (~isManeuverRunning || ~isLongManRunning)
                % Reset cooperating vehicles
                vehicleC = [];
                vehicle1 = [];
                vehicle2 = [];
                startTimer = true;
                warning('Timer Started')
                % set(cavCVisObj, 'AttValue', 'DesSpeed', cavC.Speed); 
                set(cavCVisObj, 'AttValue', 'VehType', CavCWait);
                set(cavCVisObj, 'AttValue', 'DesSpeed', cavC.Speed); 
                set(cavCVisObj, 'AttValue', 'ExtContr', 'EVC_NOT');
            end

            % Identify Vehicle 1 and Vehicle 2 on Vissim
            if ~isempty(vehicle2) && isUnderlyingType(vehicle2,'IntelligentVehicle')
                [~, cav2VisObj]= extract_vehicle_states_Vissim ...
                         (Vissim, str2double(vehicle2.VehicleID));
                if strcmp(vehicle2.DrivingBehaviour,"collaborating")
                     % Change vehicle color and type to CAVC
                     set(cav2VisObj, 'AttValue', 'VehType', Cav2Type);
                     set(cav2VisObj, 'AttValue', 'ExtContr', 4);
                end
            else
                cav2VisObj = [];
            end
            if ~isempty(vehicle1) && isUnderlyingType(vehicle2,'IntelligentVehicle')
                [~, cav1VisObj]= extract_vehicle_states_Vissim ...
                         (Vissim, str2double(vehicle1.VehicleID));
                if strcmp(vehicle1.DrivingBehaviour,"collaborating")
                     set(cav1VisObj, 'AttValue', 'VehType', Cav1Type);
                     set(cav1VisObj, 'AttValue', 'ExtContr', 4);
                end
            else
                cav1VisObj = [];
            end
            % Create array cell for vehicle state update
            vehArray = {vehicleC; vehicle1; vehicle2};
             % Create cell containing vehicle COM objects for control
            vehiclesVisObj = {cavCVisObj,cav1VisObj,cav2VisObj};
        end
    elseif startManeuver && ~startTimer 
        % Check if previous maneuver has cancelled longitudinal maneuver 
        % before execution
        if isLongManRunning
            % Update next time step for single maneuver
            isLongManRunning =  singleManeuverObj.update; 
        end
        if isLongManRunning
            % Update vehicle positions in Vissim
            for i = 1:length(vehiclesVisObj)
                if ~isempty(vehiclesVisObj{i})
                    update_vissim_states(vehArray{i},vehiclesVisObj{i});
                    if strcmp(vehArray{i}.ManeuverType,'Selfish' )
                        set(vehiclesVisObj{i}, 'AttValue', 'VehType', CavCSocial);
                    elseif strcmp(vehArray{i}.ManeuverType,'SelfishRelaxed' )
                        set(vehiclesVisObj{i}, 'AttValue', 'VehType', CavCSocialRel);
                    end
                end
            end
        else
            % Command Lane Change
            link_number = 1;
            cavC = extract_vehicle_states_Vissim (Vissim,cav_C_Vissim.No);
            cavCVisObj.MoveToLinkPosition(link_number, 2, cavC.Pos);
            % Check if vehicle has arrived
            if cavC.Lane==2
                % Return vehicle type to non collaborating
                % Change vehicle color and type to CAVC
                for i = 1:length(vehiclesVisObj)
                    if ~isempty(vehiclesVisObj{i})
                        if str2double(get(vehiclesVisObj{i}, 'AttValue', 'VehType'))~=Uncooperative
                            set(vehiclesVisObj{i}, 'AttValue', 'VehType', CavType_noLaneChange);
                            set(vehiclesVisObj{i}, 'AttValue', 'DesSpeed', 110)   ;
                        end
                        set(vehiclesVisObj{i}, 'AttValue', 'ExtContr', 'EVC_NOT');
                    end
                end
                % set flags back to false
                isManeuverRunning = false;
                startManeuver = false;
                count = count+1;
            end
        end
    elseif startManeuver && startTimer % Timer Value
        timer = timer + SampleTime;
        if timer >= max_time
            warning('timer ended')
            startTimer = false;
            timer = 0;
            startManeuver = false;
            set(cavCVisObj, 'AttValue', 'DesSpeed', 100);
        end
    end
    
    execp = {num2str(Uncooperative),num2str(CavCType),num2str(Cav2Type),num2str(Cav1Type)};
    allow_front_lane_changes (Vissim,veh_U_ID,CavType,execp);
    if VideoGeneration
        imgCount = imgCount +1;  %#ok<UNRCH>
        screenshotFileName = sprintf('%d.jpg',imgCount);
        fileName = [videoFldr filesep screenshotFileName];
        sizeFactor = 2; % 1: original size, 2: doubles size
        Vissim.Graphics.CurrentNetworkWindow.Screenshot(fileName, sizeFactor);
    end
end

% Close log files
if EstimationResults
    fclose(EstimationlogFileID);  %#ok<UNRCH>
end
if FinalResults 
    fclose(FinalResultsFileID);  %#ok<UNRCH>
    % Read CVS results
    ResultsTable = readtable([ResultsFolder, filesep, resultsFilename, '.csv']);
    % Parameters Results
    tf_array = ResultsTable.Tf;
    avgTf = mean(tf_array);
    maxTf = max(tf_array);
    minTf = min(tf_array);
    stdTf = std(tf_array);
    avgDisruption = mean(ResultsTable.Disruption);
    numManeuvers = length(tf_array);
end
% Populate Results
if FinalResults || EstimationResults 
    No = simRuns{end}.AttValue('No');
    VehTravelTime = Vissim.Net.VehicleTravelTimeMeasurements.GetAll;
    resultsSubattributes = ['(',num2str(No),',1,all)'];
    NumVehsStatic = double(VehTravelTime{1}.AttValue(['Vehs',resultsSubattributes]));
    AvgTravelTime = VehTravelTime{1}.AttValue(['TravTm',resultsSubattributes]);
    DistTravel = VehTravelTime{1}.AttValue(['DistTrav',resultsSubattributes]);
    NumVehsDynamic = count_vehicles_ahead(Vissim,veh_U_ID);
    
    fprintf(FileDescriptorID, ...
        '%d \t %s \t %.2f \t %.2f \t %.2f \t %d \t %.2f \t %d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %s \n',...
        No, TOD, StopTime, Density, CAVRelDistribution*100, NumVehsStatic, ...
        (NumVehsStatic/StopTime)*3600,NumVehsDynamic, ...
        (NumVehsDynamic/StopTime)*3600, AvgTravelTime, DistTravel, ...
        avgTf, stdTf, maxTf, minTf, avgDisruption, numManeuvers, Note);
    fclose(FileDescriptorID);  
end
Vissim.Simulation.Stop;
% Generate Video
if VideoGeneration
    writerObj = VideoWriter(strcat(ResultsFolder,filesep,name),'MPEG-4');    %#ok<UNRCH>
    writerObj.FrameRate = 10;
    open(writerObj); 
    for i=1:imgCount %where N is the number of images
        screenshotFileName = sprintf('%d.jpg',i);
        fileName = [videoFldr filesep screenshotFileName];
        I = imread(fileName); %read the next image
        writeVideo(writerObj,I); %write the image to file

    end
    close(writerObj);
    % Delete images and folder
    delete([videoFldr,'\*']);
    rmdir([videoFldr,'\']);
end



%% 
% -------------------------------------------------------------------------
% Helper Functions
% -------------------------------------------------------------------------
function create_chase_visualization2d(visObj,vehStates, pitch, zoomLevel)
    % Creates a chase plot of vehicle with respect to its states, with a
    % pitch angle and desired zoomLevel
    % ---------------------------------------------------------------------
    x1Pos=vehStates.Position(1)-zoomLevel;
    y1Pos=vehStates.Position(2)-zoomLevel;
    x2Pos=vehStates.Position(1)+zoomLevel;
    y2Pos=vehStates.Position(2)+zoomLevel;
    %orientation = vehStates.Angle+90;
    %visObj.Graphics.CurrentNetworkWindow.SetCameraPositionAndAngle...
    %    (xPos, yPos, zoomLevel, orientation, pitch);
    visObj.Graphics.CurrentNetworkWindow.ZoomTo...
        (x1Pos, y1Pos, x2Pos, y2Pos);

end
function create_chase_visualization3d(visObj,vehStates, pitch, zoomLevel)
    % Creates a chase plot of vehicle with respect to its states, with a
    % pitch angle and desired zoomLevel
    % ---------------------------------------------------------------------
    xPos=vehStates.Position(1)-40;
    yPos=vehStates.Position(2);
    orientation = vehStates.Angle+90;
    visObj.Graphics.CurrentNetworkWindow.SetCameraPositionAndAngle...
        (xPos, yPos, zoomLevel, orientation, pitch);
end

function [vehicleVisState, vehicleVisObj]= ...
    extract_vehicle_states_Vissim (visObj,vehID)
    % Extract ego vehicle COM object
    vehicleVisObj = visObj.Net.Vehicles.ItemByKey(vehID);
    vehicleVisState.No = vehID;
    % Extract ego vehicle coordinates
    egoCoords = extract_coordinates...
        (vehicleVisObj.AttValue('CoordFront'));
    vehicleVisState.Position = [egoCoords.x, egoCoords.y, egoCoords.z];
    % Extract vehicle speed
    vehicleVisState.Speed = vehicleVisObj.AttValue('Speed');
    % Extract vehicle length
    vehicleVisState.Length = vehicleVisObj.AttValue('Length');
    % Extract ego current lane
    vehicleVisState.Lane = string_lane_to_index(...
        vehicleVisObj.AttValue('Lane'));
    % Extract Ego distance travelled on link
    vehicleVisState.Pos = vehicleVisObj.AttValue('Pos');
    % Extract vehicle type
    vehicleVisState.VehType = vehicleVisObj.AttValue('VehType');
    % Extract vehicle Angle
    vehicleVisState.Angle = compute_vehicle_orientation(visObj, vehID);

end

function [neighVehStruct, neighVehVisObj] = extract_neighbor...
    (visObj, egoVehKey, neighType, attributes, distributionKey, neighbourType)
    % Extracts a neighbour from an ego vehicle (egoVehVisObj) and retturns
    % a structure containing the information of the arguments given and a
    % vehicle object from Vissim. It will look for neighbors around
    % location given a location distribution defined as part of Vissim
    % simulation with key {distributionKey}
    % Input:
    % visObj: Vissim COM object
    % egoVehKey: key identifier for Vissim Vehicle COM object containing 
    %               ego vehicle
    % neighType: string containing the Neighbor type {leader, follower, 
    %           left_leader, left_follower, right_leader, right_follower, 
    %           left_neighbors, right_neighbors}
    % distributionKey: integer containing unique distribution key
    %           identifier for location distribution (defined within vissim
    %           sim)
    % attributes: Cell array containing the attributes from Vehicle object
    %           that user requires for extraction of vehicle information.
    %           Should at least contain {'No','Lane','Pos'}
    % Returns:
    % neighVehStruct: Structure array with fields {attributes} containing
    %           information about neighbor vehicles
    % neighVehVisObj: A cell array containing Vissim COM IVehicle objects
    %           of neighbors.
    % ---------------------------------------------------------------------
    arguments
        visObj 
        egoVehKey 
        neighType (1,:) char {mustBeMember(neighType,...
            {'leader','follower','left_leader','left_follower',...
            'right_follower','right_leader',...
            'left_neighbors','right_neighbors', 'front_leaders'})}
        attributes {mustBeUnderlyingType(attributes, 'cell')}
        distributionKey 
        neighbourType  (1,:) char {mustBeMember(neighbourType,...
            {'DistanceDist','Link','Absolute'})} = 'Link'
    end
    % Extract ego vehicle COM object
    egoVehicleVisObj = visObj.Net.Vehicles.ItemByKey(egoVehKey);
    % Extract ego vehicle coordinates
    egoCoords = extract_coordinates...
        (egoVehicleVisObj.AttValue('CoordFront'));
    % Extract ego current lane
    egoLane = string_lane_to_index(egoVehicleVisObj.AttValue('Lane'));
    egoPos = egoVehicleVisObj.AttValue('Pos');

    % Extract vehicles in neighborhood
    [candidateNeighVehStruct, ~] = get_neighborhood_vehicles...
        (visObj, egoCoords.x,egoCoords.y, attributes, distributionKey, neighbourType);
    
    % Delete ego vehicle from structure array
    if ~isfield(candidateNeighVehStruct,'No')
        error('"No" should be part of the attributes')
    end
    candidateNeighVehStruct = ...
        candidateNeighVehStruct([candidateNeighVehStruct.No]~=egoVehKey);
    % Check if there are vehicles remaining
    if size(candidateNeighVehStruct,1)<1
        warning('No neighbors were found')
        neighVehStruct = candidateNeighVehStruct;        
        neighVehVisObj = form_vehicle_obj_array(visObj, [neighVehStruct.No]);
        return
    end
    % Extract Neighbor given the type
    switch neighType
        case {'leader','Leader'}
            % Extract vehicles in current lane
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Lane]==egoLane);
            % Filter vehicles only in front of ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Pos]>=egoPos);
            % Look for closest vehicle
            [~, minIndex] = min([candidateNeighVehStruct.Pos]);
            candidateNeighVehStruct = candidateNeighVehStruct(minIndex);
        case {'follower','Follower'}
           % Extract vehicles in current lane
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Lane]==egoLane);
            % Filter vehicles only behind of ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Pos]<egoPos);
            % Look for closest vehicle
            [~, maxIndex] = max([candidateNeighVehStruct.Pos]);
            candidateNeighVehStruct = candidateNeighVehStruct(maxIndex);
        case {'left_leader', 'right_leader'}
            % Determine if left or right leader
            if strcmp(strtok(neighType,'_'),'left')
                laneCmp = egoLane+1;
            elseif strcmp(strtok(neighType,'_'),'right')
                laneCmp = egoLane-1;
            else
                error('type not recognized')
            end
            % Extract vehicles on the adjacent lane with respect to ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Lane]==laneCmp);
            % Filter vehicles only in front of ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Pos]>=egoPos);
            % Look for closest vehicle
            [~, minIndex] = min([candidateNeighVehStruct.Pos]);
            candidateNeighVehStruct = candidateNeighVehStruct(minIndex);
        case {'left_follower', 'right_follower'}
            % Determine if left or right leader
            if strcmp(strtok(neighType,'_'),'left')
                laneCmp = egoLane+1;
            elseif strcmp(strtok(neighType,'_'),'right')
                laneCmp = egoLane-1;
            else
                error('type not recognized')
            end
            % Extract vehicles on the adjacent lane with respect to ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Lane]==laneCmp);
            % Filter vehicles only behind of ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Pos]<egoPos);
            % Look for closest vehicle
            [~, maxIndex] = max([candidateNeighVehStruct.Pos]);
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(maxIndex);
        case {'left_neighbors','right_neighbors'}
            % Determine if left or right leader
            if strcmp(strtok(neighType,'_'),'left')
                laneCmp = egoLane+1;
            elseif strcmp(strtok(neighType,'_'),'right')
                laneCmp = egoLane-1;
            else
                error('type not recognized')
            end
            % Extract vehicles on the adjacent lane with respect to ego
            candidateNeighVehStruct = candidateNeighVehStruct(...
                [candidateNeighVehStruct.Lane]==laneCmp);
        case 'front_leaders'
            % Filter vehicles only in front of ego
            candidateNeighVehStruct = ...
                candidateNeighVehStruct(...
                [candidateNeighVehStruct.Pos]>=egoPos);

    end
    % Create output arrays
    neighVehStruct = candidateNeighVehStruct;
    neighVehVisObj = form_vehicle_obj_array(visObj, [neighVehStruct.No]);    
end

function [numberVehiclesAhead, averageSpeedVehAhead] = ...
    count_vehicles_ahead(visObj,targetVehiclekey)
%count_vehicles_ahead:
% counts vehicles only in front of targetVehicle. Additionally, it measures
% the current average speed of the vehicles ahead
% -------------------------------------------------------------------------
    % Define attributes for lookup
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane';'Length'};
    % Extract vehicle leaders
     neighVehVisStruct= extract_neighbor...
    (visObj, targetVehiclekey, 'front_leaders', attributes, 1);
    
    averageSpeedVehAhead =  mean([neighVehVisStruct.Speed]);
    % Extracct Vehicle Count
    numberVehiclesAhead = length([neighVehVisStruct.Speed]);
end

function allow_front_lane_changes (visObj,targetVehiclekey,front_vehicle_type, vehTypeExceptions)
%allow_front_lane_changes:
% finds vehicles in front of targetVehicle and changes vehicle type of
% front vehicles to "front_vehicle_type" so that lane changes are allowed
% again passed the targetVehicle. Vehicle type is just a number that refers
% to a defined value within simulation on Vissim
% -------------------------------------------------------------------------
    % Define attributes for lookup
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane';'Length'};
    % Extract vehicle leaders
     [neighVehVisStruct, neighVehVisObj] = extract_neighbor...
    (visObj, targetVehiclekey, 'front_leaders', attributes, 1);
    % Change vehicle type
    for i = 1:length(neighVehVisObj)
        if ~any(strcmp(neighVehVisStruct(i).VehType,vehTypeExceptions))
            set(neighVehVisObj(i), 'AttValue', 'VehType', front_vehicle_type);
        end
    end

end
function [candVissim, candCAV] = find_cav_candidates_number(visObj, vehicleU, ...
    maxCandidates, scenario, sampleTime, stopTime, constraints)
% find_cav1:
% Extracts neighbors in the vecinity of vehicle U and decides whiich
% vehicle corresponds to CAV1. There are 2 possible candidates (left
% follower and left leader. CAV1 is defined as the first vehicle in front
% of relDistU from posU-Vehicle U length
% Return:
% cav1Vis - structure representing vehicle vissim states
% cav1_VisObj - Vissim IVehicle COM object representing vehicle
% -------------------------------------------------------------------------
    
    % Preallocate cell arrays
    candidates_VisObj = cell(maxCandidates+1,1);
    % Define attributes to be extracted
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane';'Length'};
    % Extract vehicles behind
    for i = 1:maxCandidates+1
        if i==1
            candidate = extract_neighbor(visObj, vehicleU.No,...
                'left_leader', attributes, 1);
        else
            candidate = extract_neighbor(visObj, candVissim(i-1).No,...
            'follower', attributes, 1);
        end
        [candVissim(i), candidates_VisObj{i}]= ...
            extract_vehicle_states_Vissim(visObj,candidate.No); %#ok<AGROW> 
        % Create Intelligent vehicles array
        candCAV(i) = create_cav (scenario, candVissim(i),'CAV', ...
            sampleTime, stopTime, constraints); %#ok<AGROW> 
    end

end

function [candVissim, candCAV] = find_cav_candidates_distance(visObj,vehicleC, vehicleU, ...
    scenario, sampleTime, stopTime, constraints, uncooperativeType)
% find_cav1:
% Extracts neighbors in the vecinity of vehicle U and decides whiich
% vehicle corresponds to CAV1. There are 2 possible candidates (left
% follower and left leader. CAV1 is defined as the first vehicle in front
% of relDistU from posU-Vehicle U length
% Return:
% cav1Vis - structure representing vehicle vissim states
% cav1_VisObj - Vissim IVehicle COM object representing vehicle
% -------------------------------------------------------------------------
    % Extract maximum look behind distance from CAVC
    distCRear = constraints.deltaCavCRear;
    distUFront = constraints.deltaCavUFront;
    rearOverhang = 1;
    CavCDistance = (vehicleC.Pos-vehicleC.Length-distCRear);
    % Preallocate cell arrays
    candidates_VisObj = {};
    % Define attributes to be extracted
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane';'Length'};
    % Compute initial relative Difference
    count = 1;
    while true
        if count==1
            candidates = flip(extract_neighbor(visObj, vehicleU.No,...
                'left_neighbors', attributes, 1));
            % Extract first vehicle in front of front reference line
            candidates = candidates(...
                [candidates.Pos]-(vehicleU.Pos+distUFront)>0);
            [~, candIndex]=sort([candidates.Pos]);
            if ~isempty(candIndex)
                candidate = candidates(candIndex(1));
            else
                candidate = extract_neighbor(visObj, vehicleU.No,...
                    'left_leader', attributes, 1);
            end
        else
            candidate = extract_neighbor(visObj, candVissim(count-1).No,...
            'follower', attributes, 1);
        end
        % Compute relative Difference
        relDifference = candidate.Pos(1)-...
            candidate.Length+rearOverhang - CavCDistance;
        % Check if ahead of C
        if relDifference >= 0
            % Extract vehicles behind
            [candVissim(count), candidates_VisObj{count}]= ...
                extract_vehicle_states_Vissim(visObj,candidate.No); %#ok<AGROW> 
            % Create Intelligent vehicles array
            if any(str2double(candVissim(count).VehType)==uncooperativeType)
                cooperationType = 'NonControlled';
            else
                cooperationType = 'CAV';
            end
            candCAV(count) = create_cav (scenario, candVissim(count),...
                cooperationType, sampleTime, stopTime, constraints); %#ok<AGROW> 
        else
             [candVissim(count), candidates_VisObj{count}]= ...
                extract_vehicle_states_Vissim(visObj,candidate.No); %#ok<AGROW> 
            % Create Intelligent vehicles array
            if any(str2double(candVissim(count).VehType)==uncooperativeType)
                cooperationType = 'NonControlled';
            else
                cooperationType = 'CAV';
            end
            candCAV(count) = create_cav (scenario, candVissim(count),...
                cooperationType, sampleTime, stopTime, constraints); %#ok<AGROW>
            break
        end
        % Increase counter
        count = count + 1;
    end

end


function [cav1, cav1_VisObj] = find_cav1(visObj, vehicleU, relDistU) 
% find_cav1:
% Extracts neighbors in the vecinity of vehicle U and decides whiich
% vehicle corresponds to CAV1. There are 2 possible candidates (left
% follower and left leader. CAV1 is defined as the first vehicle in front
% of relDistU from posU-Vehicle U length
% Return:
% cav1Vis - structure representing vehicle vissim states
% cav1_VisObj - Vissim IVehicle COM object representing vehicle
% -------------------------------------------------------------------------
    rearOverhang = 1;
    % Define attributes to be extracted
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane';'Length'};
    % Extract initial candidate with its position
    leader = extract_neighbor(visObj, vehicleU.No,...
        'left_leader', attributes, 1);
    if isempty(leader)
        leader = extract_neighbor(visObj, vehicleU.No,...
            'left_follower', attributes, 1);
    end
    cav1_candidate = leader;
    % Create a vector placeholder
    relDifference = cav1_candidate.Pos(1)-...
        cav1_candidate.Length+rearOverhang - ...
        (vehicleU.Pos-vehicleU.Length-relDistU);
    % Index indicating the number of searchs performed
    numSearches = 1; 
    while relDifference >= 0
        % extract follower from vehicle with index i
        candidate = extract_neighbor(visObj, cav1_candidate(end).No,...
        'follower', attributes, 1);
        %Compute difference
        relDifference = candidate.Pos-candidate.Length+rearOverhang - ...
            (vehicleU.Pos-vehicleU.Length-relDistU);
        % Store and advance if relDifference >=0
        if relDifference >= 0
            numSearches = numSearches + 1;
            cav1_candidate(numSearches) = candidate;  
        else
            break
        end
    end
    
    % Extract cav1 as the first vehicle in front of reference line
    cav1 = cav1_candidate(end);
    % Compute viissiim vehiicle states
    [cav1, cav1_VisObj]= extract_vehicle_states_Vissim (visObj,cav1.No);
end
function [cav2, cav2_VisObj] = find_cav2(visObj,cav1,cavC)
% find_cav2:
% Defines CAV2 as the first vehicle behind cav1 if cav1 exists. Otherwise,
% it is deined as the left leader of cavC or the left left follower of cavC

% giving priority to left leader if it exists.
% Return:
% cav2Vis - structure representing vehicle vissim states
% cav2_VisObj - Vissim IVehicle COM object representing vehicle
% -------------------------------------------------------------------------
    % Define attributes to be extracted
    attributes = {'No'; 'VehType'; 'Speed'; 'Pos'; 'Lane'};
    % Extract initial candidate with its position
    if ~isempty(cav1)
        cav2 = extract_neighbor(visObj, cav1.No,'follower', attributes, 1);
        % Compute viissiim vehiicle states
        [cav2, cav2_VisObj]= extract_vehicle_states_Vissim (visObj,cav2.No);
    else
        cav2_candidate_1 = extract_neighbor ...
            (visObj, cavC.No,'left_leader', attributes, 1);
        cav2_candidate_2 = extract_neighbor ...
            (visObj, cavC.No,'left_follower', attributes, 1);
        if ~isempty(cav2_candidate_1) 
             [cav2, cav2_VisObj]= extract_vehicle_states_Vissim (visObj,cav2_candidate_1.No);
        elseif isempty(cav2_candidate_1) && ~isempty(cav2_candidate_2)
            [cav2, cav2_VisObj]= extract_vehicle_states_Vissim (visObj,cav2_candidate_2.No);
        else
            cav2 = repmat(cavC,0,0); 
            cav2_VisObj = [];
        end
    end
end

function vehicle = create_cav (scenario,vehStruct,vehType, sampleTime, ...
    stopTime, constraints)
% intelVehicle:
% Creates a class of intelligent Vehicle that defines the dynamics and
% collaborates with other cavs
%% 
% Input:
% vehID - Vehicle ID [string]
% position - 2D position vector for initialization [m m]
% speed - vehicle speed [m/s]
% vehType - options = {'CAV', 'NonControlled'}
% sampleTime: Simulation sample time, defines the granularity for
%   integration [sec]
% currTime: simulation current time
% Output:
% intelVehicle - vehicle object of type @IntelligentVehicle
% -------------------------------------------------------------------------
     global params
    %Extract vehicle dimensions
    length = vehStruct.Length;
    rearOverhang = 1; 
    % Adjust vehicle center to DSD
    vehStruct.Pos(1)  = vehStruct.Pos(1)-length+rearOverhang;
    % Create initial conditions structure
    laneWidth = -params.road.laneWidth;
    if vehStruct.Lane == 1
        vehStruct.Position(2) = -laneWidth-laneWidth/2;
    elseif vehStruct.Lane == 2
        vehStruct.Position(2) = -laneWidth/2;
    end
    
    vehInitCond.Position = [vehStruct.Pos vehStruct.Position(2)]'; %[x,y][m]
    vehInitCond.Velocity = [vehStruct.Speed/3.6, 0]'; %[vx,vy][m/s]
    vehInitCond.Heading = 0; %[deg]
    vehInitCond.Steering = 0; %[deg]
    % Create intelligent vehicle
    vehicle = IntelligentVehicle(num2str(vehStruct.No),scenario,...
        vehInitCond, stopTime, constraints,...
            'VehicleType',vehType,'SampleTime', sampleTime,...
            'SafetyDistance', constraints.safetyDist,'VehicleLength',length,...
            'VehicleRearOverhang',rearOverhang);
end

function hasBeenUpdated = update_vissim_states(intelVehicle,vehVisObj)
% Updates vehicle states computed on DSD and comands them to
% sumo using traci. Additionally, in order to compute the
% correct command for traci, a logic to determine current edge
% and lane index is performed
% -------------------------------------------------------------
    % Update Speeds
    new_desspeed = intelVehicle.CurrentState.Velocity(1);
    if new_desspeed < 10
        new_desspeed = 10;
    end
    set(vehVisObj, 'AttValue', 'Speed', new_desspeed*3.6);
    % check if vehicle is on current lane
    hasBeenUpdated = true;
    % Update Vehicle Position
    lane_number = intelVehicle.Vehicle.currentLane;
    if lane_number == 4
        lane_number = 1;
    elseif lane_number == 3
        lane_number=2;
    end
    link_number = 1;
    frontOverhang = intelVehicle.Vehicle.Length - intelVehicle.Vehicle.RearOverhang;
    link_coordinate = intelVehicle.Vehicle.Position(1)+frontOverhang;
    vehVisObj.MoveToLinkPosition(link_number, lane_number, link_coordinate)

end % update_sumo_states
%% 
% -------------------------------------------------------------------------
% UTILS FUNCTIONS
% -------------------------------------------------------------------------
function coordinates = extract_coordinates(string)
% Extracts coordinates from string with delimiters ' '
% -------------------------------------------------------------------------
    [x, rest] = strtok(string, ' ');
    [y, z] = strtok(rest, ' ');
    coordinates.x = str2double(x);
    coordinates.y = str2double(y);
    coordinates.z = str2double(z);
end

function vehicleComObjects = form_vehicle_obj_array(visObj, keysArray)
% Return a cell array containing IVehicleCOMObj
% -------------------------------------------------------------------------
    vehicleComObjects = cell(0);
    for i=1:length(keysArray)
        veh = visObj.Net.Vehicles.ItemByKey(keysArray(i));
        vehicleComObjects = cat(1,vehicleComObjects, veh);
    end
end

function angle = compute_vehicle_orientation(visObj, vehID)
    % Computes the orientation of the vehicle by extracting the front and
    % rear coordinates of vehicles
    % ---------------------------------------------------------------------
    % Extract vehicle COM object
    vehicleVisObj = visObj.Net.Vehicles.ItemByKey(vehID);
    frontCoord = extract_coordinates(vehicleVisObj.AttValue('CoordFront'));
    rearCoord = extract_coordinates(vehicleVisObj.AttValue('CoordRear'));
    deltaX = frontCoord.x - rearCoord.x;
    deltaY = frontCoord.y - rearCoord.y;
    % compute angle [deg]
    angle = rad2deg(atan(deltaY/deltaX));
end

function lane = string_lane_to_index (string)
    [~, lane] = strtok(string, ' - ');
    lane = strtok(lane, ' - ');
    if iscell(lane)
        lane = cellfun(@str2double,lane);
    else
        lane = str2double(lane);
    end
end

function [neighbourStruct, neighVehVisObj]= get_neighborhood_vehicles...
    (visObj, xCoord,yCoord, attributes, typeKey, neighbourType)
    % Returns a set of vehicles around a location given a specified
    % location distribution defined inside of Vissim. Additionally, it
    % returns an IVehicleCollection of vehicles in the neighborhood
    % ---------------------------------------------------------------------
    arguments
        visObj 
        xCoord (1,1) {mustBeNumeric}
        yCoord (1,1) {mustBeNumeric}
        attributes {mustBeUnderlyingType(attributes, 'cell')}
        typeKey
        neighbourType  (1,:) char {mustBeMember(neighbourType,{'DistanceDist','Link','Absolute'})} = 'Absolute'
    end
    % Get all vehicles around specified location
    switch neighbourType
        case 'DistanceDist'
            neighVehVisObj = visObj.Net.Vehicles.GetByLocation...
                (xCoord,yCoord,typeKey);
            allNeighVehAttributes = neighVehVisObj.GetMultipleAttributes(attributes);
        case 'Link'
            neighVehVisObj = visObj.Net.Links.ItemByKey(typeKey).Vehs;
            allNeighVehAttributes = neighVehVisObj.GetMultipleAttributes(attributes);
        case 'Absolute'
            neighVehVisObj = visObj.Net.Vehicles.GetAll;
        allNeighVehAttributes = visObj.Net.Vehicles.GetMultipleAttributes(attributes);
        otherwise
            error('neighbourType search must be Link, DistanceDist, or Absolute')
    end
    % Convert lanes to index
    laneIndex = strcmp(attributes,'Lane');
    allNeighVehAttributes(:,laneIndex) = ...
        num2cell(string_lane_to_index (allNeighVehAttributes(:,laneIndex)));
    % Convert cell array to structure
    neighbourStruct = cell2struct(allNeighVehAttributes,attributes,2);
end


function create_log_headers(fileID)
    % CAV C
    fprintf(fileID, 'CAVC_ID \t CAVC_Type \t X_C_0 [m] \t V_C_0 [m/s] \t X_C_f [m] \t V_C_f \t DELTA_X_C [m^2] \t ');
    % CAV 1
    fprintf(fileID, 'CAV1_ID \t CAV1_Type \t X_1_0 [m] \t V_1_0 [m/s] \t X_1_f [m] \t V_1_f \t DELTA_X_1 [m^2] \t ');
    % CAV 2
    fprintf(fileID, 'CAV2_ID \t CAV2_Type \t X_2_0 [m] \t V_2_0 [m/s] \t X_2_f [m] \t V_2_f \t DELTA_X_2 [m^2] \t ');
    % Maneuver
    fprintf(fileID, 'Tf \t Disruption \t TotEnergy \t FlowSpeed \t ManeuverType \n');
end

function create_results_descriptor_headers(fileID)
    % CAV C
    fprintf(fileID,'No \t TOD [mm-dd-yyyy_HH-MM] \t SimLength [s] \t Density [veh/hour] \t Penetration [%%] \t NumVeh (Static) \t Throghput(Static) [v/h] \t NumVeh (Dyn) \t Throughput(Dyn) [v/h] \t AvgTravelTime [s] \t DistTravel [m] \t AvgTf [s] \t StdTf [s] \t MaxTf [s] \t MinTf [s] \t AvgDisruption \t NumManeuvers \t Comments \n');
end

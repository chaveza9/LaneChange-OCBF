function scenario = ds4vehicleScenario(parameters)
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.10 (R2021a) and Automated Driving Toolbox 3.3 (R2021a).
% Generated on: 27-Apr-2021 11:52:58

% Extract road Parameters
p = parameters;
% --------------------- Construct a drivingScenario object. ---------------
scenario = drivingScenario;
% Add all road segments
roadCenters = [0 0 0;
               p.road.length 0 0];
laneSpecification = lanespec(4, 'Width', p.road.laneWidth);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
end



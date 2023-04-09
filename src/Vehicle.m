classdef Vehicle < Control.OCBF
    %HDV Summary of this class goes here
    %   Detailed explanation goes here
    properties (SetAccess = protected, GetAccess = public)
        % Vehicle Metadata
        % Vehicle name
        VehicleID (1,:) char {mustBeText}
        % Vehicle type (is manned or non manned)
        VehicleType (1,:) char {mustBeMember(VehicleType,{'NonControlled','CAV'})} = 'NonControlled'
        % Vehicle Initial conditions
        InitialConditions =  struct('Position',[],...
            'Velocity',[],...
            'Heading',[],...
            'Steering',[])
        % VehicleConstrints
        AccelMax
        AccelMin
        VelMax
        VelMin
        Selfish = false;
        DesSpeed = 0;
        % Rear Safety Distance
        SafetyDistance (1,:) double {mustBeNumeric, mustBePositive}= 10;
        % Communication range (m)
        CommunicationRange (1,:) double {mustBeNumeric, mustBePositive}= 250;
        % Simulation Metadata
        % Simulation sample time
        SampleTime double {mustBeNonnegative, mustBeNumeric} = 0.01;
        % Simulation current time
        CurrentTime double {mustBeNonnegative, mustBeNumeric} = 0.0;
        % Stop Time
        StopTime double {mustBeNonnegative, mustBeNumeric} = 100;
        % Operational Parameters
        % Vehicle Dynamics
        Maneuver  % Vehicle Dynamics
        CavHistory = [];
        % Vehicle driving behaviour
        DrivingBehaviour (1,:) char {mustBeMember(DrivingBehaviour,{'acc','collaborating','selfless' })} = 'selfless'
        % Vehicle states
        CurrentState = struct('Position',[],...
            'Velocity',[],...
            'Heading',[],...
            'Steering',[])

        % Collaborating Request
        hasReceivedRequest logical {mustBeNumericOrLogical} =  false;
        % Vehicle roll if CAV once request has been set
        RollType (1,:) char {mustBeMember(RollType,{'none','cav1','cav2','cavC'})} = 'none';
        ManeuverType {mustBeMember(ManeuverType,{'Accel','Decel','Social','Selfish', 'SelfishRelaxed','SelfishHDVs', 'none'})} = 'none';
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
        % HDV2 parameters
        isHDV2 = false;
        CAVC_ID;
        CAV1_ID;
        IntersectTime = [];
    end
    
    methods
        function self = Vehicle(dt, H, substeps, x_goal, r, Q, R, ...
                speed_lims, cntrl_lims, static_obstacle)
            %MPC_CBF Construct an instance of this class
            arguments
                dt
                H
                substeps
                x_goal
                r
                Q = eye(4)
                R = eye(2)
                speed_lims = [0.01, 2]
                cntrl_lims = [1, 0.5]
                static_obstacle {mustBeNumericOrLogical} = 1
            end
            %  Initialize superclass
            self@CAV.OCBF(dt, H, substeps, x_goal, r, speed_lims, cntrl_lims);
            self.Q = Q;
            self.R = R;
            % Initialize problem
            [self.prob, self.x0_var, self.u0_var, self.x_var, self.u_var,...
                self.x_obs_par] = self.define_mpc(static_obstacle);
        end
        %% Define Control Affine Dynamics
        function x_dot = f(x)
            x_dot = [x(3) * cos(x(4)), x(3) * sin(x(4)), 0, 0]';
        end
        function  val = g(~)
             val = [zeros(2,2);
                    eye(2)];
        end
        
    end

end


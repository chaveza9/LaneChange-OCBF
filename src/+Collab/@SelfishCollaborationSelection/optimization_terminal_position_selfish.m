%terminal position optimization problem
%x(1):x_1f;x(2):x_Bf;x(3):x_3f
function [Xf_1, Xf_2, Xf_C, fval, flag] =optimization_terminal_position_selfish(veh1, veh2, vehC, xcf, vehU_2, t_f, params)

%% General Parameters Extraction
u_max = params.u_max;    % Vehicle i max acceleration
% u_min = params.u_min;   % Vehicle i min acceleration
v_min = params.v_min; % Vehicle i min velocity
v_max = params.v_max;   % Vehicle i max velocity
% safetyDist = params.safetyDist;
%% Define Optimization case
if veh1.Collaborating && veh2.Collaborating
    judge = 1;
elseif veh1.Collaborating && ~veh2.Collaborating
    judge = 2;
elseif ~veh1.Collaborating && veh2.Collaborating
    judge = 3;
end    
%% vehicle 1 initial configuration
x_1_0 = veh1.Pose(1);     % Vehicle 1 initial position
v_1_0 = hypot(veh1.Pose(3), veh1.Pose(4));     % Vehicle 1 initial velocity
%% vehicle 2 initial configuration
x_2_0 = veh2.Pose(1);     % Vehicle 1 initial position
v_2_0 = hypot(veh2.Pose(3), veh2.Pose(4));     % Vehicle 1 initial velocity
%% vehicle U_2 initial configuration
x_U_2_0 = vehU_2.Pose(1);     % Vehicle U2 initial position
v_U_2_0 = hypot(vehU_2.Pose(3), vehU_2.Pose(4));     % Vehicle U2 initial velocity
%% vehicle C initial configuration
% x_C_0 = vehC.Pose(1);     % Vehicle C initial position
v_C_0 = hypot(vehC.Pose(3), vehC.Pose(4));     % Vehicle C initial velocity
Xf_C = xcf;     % Vehicle C initial position
%% Optimization weights
alpha = 0.99;
% Headway
theta = 0.5;
% Define Options Optimization
options = optimoptions("fmincon",...
    "Algorithm","interior-point",...
    "EnableFeasibilityMode",true,...
    "SubproblemAlgorithm","cg", "Display","none");
%% Optimization Problem definition
switch judge
    case 1 % CAV 1 and CAV 2 collaborate
        % Objective function definition
        fun = @(x)(1-alpha)*(x(1)-x_1_0-v_1_0*t_f)^2+... % Delta_1
            alpha*(x(2)-x_2_0-v_2_0*t_f)^2;    % Delta_2
        % Initial Condition Computation
        x0 = [x_1_0+v_1_0*t_f,x_2_0+v_2_0*t_f];
        %  Linear Constraints Definition
        A = -[1 0; %1-c
             0 -1; %c-2
            -1 0]; %u2-1
        safetyDist_2 = v_2_0*theta;
        safetyDist_C = (v_C_0+u_max*t_f)*theta;
        safetyDist_1 = (v_1_0+u_max*t_f)*theta;
        b = -[safetyDist_C+veh1.RearOverhang+vehC.FrontOverhang+ Xf_C;                        % x1-xc >= dc(vc(t))
              safetyDist_2+veh2.RearOverhang+vehC.FrontOverhang - Xf_C;                       % xc-x2 >= d2(d2(t))
              safetyDist_1+vehU_2.RearOverhang+veh1.FrontOverhang - (x_U_2_0+t_f*v_U_2_0)];   % xu_1-x1 >= d1(v1(t))
        % Boundary conditions
        lb = [ x_1_0+v_min*t_f, x_2_0+v_min*t_f];
        ub = [x_1_0+v_max*t_f, x_2_0+v_max*t_f];
        % Equality constraints
        Aeq=[];
        beq=[];
        % Solve optimization problem
        [xf, fval, flag] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,[],options);
        %Decompose Results and create bus
        Xf_1 = xf(1);
        Xf_2 = xf(2);
%         if v_2_0<=23
%             fval=Inf;
%         end
    case 2 % only CAV 1 collaborates
        % Objective function definition
        fun = @(x)(x-x_1_0-v_1_0*t_f)^2; % Delta_1
        % Initial Condition Computation
        x0 = x_1_0+v_1_0*t_f;
        %  Linear Constraints Definition
        A = -[1 ; %1-c
             -1]; %u2-1
        safetyDist_C = (v_C_0+u_max*t_f)*theta;
        safetyDist_1 = (v_1_0+u_max*t_f)*theta;
        b = -[safetyDist_C+veh1.RearOverhang+vehC.FrontOverhang+ Xf_C;                        % x1-xc >= dc(vc(t))
              safetyDist_1+vehU_2.RearOverhang+veh1.FrontOverhang - (x_U_2_0+t_f*v_U_2_0)];   % xu_1-x1 >= d1(v1(t))
        % Boundary conditions
        lb = x_1_0+v_min*t_f;
        ub = x_1_0+v_max*t_f;
        % Equality constraints
        Aeq=[];
        beq=[];
        % Solve optimization problem
        [xf, fval, flag] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,[],options);
        fval = (1-alpha)*fval;
        %Decompose Results and create bus
        Xf_2 = x_2_0+v_2_0*t_f;
        Xf_1 = xf;
    case 3 % only CAV 2 collaborates
        % Objective function definition
        fun = @(x) (x-x_2_0-v_2_0*t_f)^2;    % Delta_2
        % Initial Condition Computation
        x0 = x_2_0+v_2_0*t_f;
        %  Linear Constraints Definition
        A = 1; %c-2
        safetyDist_2 = v_2_0*theta;
        b = -(safetyDist_2+veh2.RearOverhang+vehC.FrontOverhang - Xf_C);                       % xc-x2 >= d2(d2(t))
        % Boundary conditions
        lb = x_2_0+v_min*t_f;
        ub = x_2_0+v_max*t_f;
        % Equality constraints
        Aeq=[];
        beq=[];
        % Solve optimization problem
        [xf, fval, flag] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,[],options);
        %Decompose Results and create bus
        Xf_1 = x_1_0+v_1_0*t_f;
        Xf_2 = xf;
end
% %% Initial Condition Computation
% x0 = [x_1_0+v_1_0*t_f,x_2_0+v_2_0*t_f];
% %% Compute optimization problem
% % Objective function definition
% fun = @(x)(1-alpha)*(x(1)-x_1_0-v_1_0*t_f)^2+... % Delta_1
%        alpha*(x(2)-x_2_0-v_2_0*t_f)^2;    % Delta_2
% % Constraints definition
% %     x = (x1_f, x2_f]
% A = -[1 0; %1-c
%     0 -1; %c-2
%     -1 0]; %u2-1
% %     b = -[safetyDist+veh1.RearOverhang+vehC.FrontOverhang+ Xf_C;                           %xc-x2 >= d2(v2(t))
% %           safetyDist+veh2.RearOverhang+vehC.FrontOverhang - Xf_C;   %xu_1-xc >= dc(vc(t))
% %           safetyDist+vehU_2.RearOverhang+veh1.FrontOverhang - (x_U_2_0+t_f*v_U_2_0)];  %xu_2 - xc >= d1(d1(t))
% safetyDist_2 = v_2_0*theta;
% safetyDist_C = (v_C_0+u_max*t_f)*theta;
% safetyDist_1 = (v_1_0+u_max*t_f)*theta;
% %     safetyDist_2 = 20;
% %     safetyDist_C = 20;
% %     safetyDist_1 = 20;
% b = -[safetyDist_C+veh1.RearOverhang+vehC.FrontOverhang+ Xf_C;                           %xc-x2 >= d2(v2(t))
%     safetyDist_2+veh2.RearOverhang+vehC.FrontOverhang - Xf_C;                       %xu_1-xc >= dc(vc(t))
%     safetyDist_1+vehU_2.RearOverhang+veh1.FrontOverhang - (x_U_2_0+t_f*v_U_2_0)];  %xu_2 - xc >= d1(d1(t))
% Aeq=[];
% beq=[];
% 
% lb = [ x_1_0+v_min*t_f, x_2_0+v_min*t_f];
% ub = [x_1_0+v_max*t_f, x_2_0+v_max*t_f];
% %results
% %     options = optimoptions('fmincon','Algorithm','interior-point','MaxIterations',3000,"Display","notify-detailed");
% options = optimoptions("fmincon",...
%     "Algorithm","interior-point",...
%     "EnableFeasibilityMode",true,...
%     "SubproblemAlgorithm","cg", "Display","notify-detailed");
% [xf, fval, flag] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,[],options);
% %     fval = fval+(Xf_C-x_C_0).^2;
% %Decompose Results and create bus
% Xf_1 = xf(1);
% Xf_2 = xf(2);

end
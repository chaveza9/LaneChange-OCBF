function [acc,v_b,v_a] = gippsDriverModel(spacing,speed,speedDiff,options)
arguments
    spacing
    speed
    speedDiff
    options.desiredSpeed = 30;
    options.maxSpeed = 33;
    options.minSpeed = 15;
    options.minAcc = -7;
    options.maxAcc = 3;
    options.reactionTime = 0.6;

end

% Copyright 2020 - 2020 The MathWorks, Inc.

desiredSpeed = options.desiredSpeed; %Desired speed
maxSpeed = options.maxSpeed; %max. speed
minSpeed = options.minSpeed; % min. speed
minAcc = options.minAcc ; %min. acceleration
maxAcc = options.maxAcc; %max. acceleration
minAccEstimate = minAcc;
reactionTime = options.reactionTime;
S0 = 2;

v_now = speed;
v_infront = speed+speedDiff;

v_a = v_now + 2.5*maxAcc*reactionTime*(1-v_now/desiredSpeed)*sqrt(0.025+v_now/desiredSpeed);
v_b = minAcc*reactionTime + sqrt((minAcc*reactionTime)^2-minAcc*(2*(spacing-S0)-v_now*reactionTime-v_infront^2/minAccEstimate));

v_b = max(v_b,minSpeed);
v_a = min(v_a,maxSpeed);
v_new = min(v_a,v_b);

acc = (v_new-v_now)/reactionTime;

end
function params = turtlebotcontrol

    params.Ts = 0.1;          % Sample time

    params.bufSize = 5;        % Filter buffer size
    params.maxDisp = 50;       % Max object displacement [pixels]
    params.minSize = 10;       % Min object size [pixels]
    params.maxSize = 400;      % Max object size [pixels]
    params.maxCounts = 3;      % Max outlier counts before stopping

    params.linVelGain = 1e-3;  % Linear control gain
    params.angVelGain = 4e-3;  % Angular control gain
    params.maxLinVel = 0.1;   % Max linear speed
    params.maxAngVel = 0.1;   % Max angular speed

    params.posDeadZone = 30;   % Steering control marker position dead zone [pixels] 
    params.targetSize = 60;   % Linear speed control target blob size [pixels]
    params.sizeDeadZone = 30;  % Linear speed control size dead zone [pixels]
    params.speedRedSize = 100; % Minimum pixel value before turning speed is ramped down
   
end
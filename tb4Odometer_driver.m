%% Clear variables and restart workspace
clear;

proj_startup;

%% get messages

scans = getScans(bagFile);
imu = getIMU(bagFile);
vlr = getWheelVels(bagFile);
tickslr = getTicks(bagFile);
robotOdom = getOdoms(bagFile);

TT = synchronize(scans, imu, vlr, tickslr, robotOdom);

N = numel(TT.Time);

iters = [];
for i = 1:N
    if(sum(ismissing(TT(i,:))) > 0)
        continue;
    end
    iters = [iters i];
end
%%
TT = TT(iters,:);
%% Create Odometer Object
odom = tb4Odometer;

% z = [V_l; V_r; theta_imu; V; omega_z];

% S = [x; y; theta; V; W];

Q = diag([1e-2 1e-2 1e-2 1e3 1e-3]);
R = diag([1e-3 1e-3 1e-1 1e-2 1e-2]);
% odom = setQ(odom, Q);
odom = setR(odom, R);

poses = EstimatePoses(odom, TT);

plot(poses.x.Data)
hold on;
plot(TT.Xlaser)


%%


function res = EstimatePoses(odom, TT)

time_ = seconds(TT.Time);
ticksl_ = TT.TicksL;
ticksr_ = TT.TicksR;

vl_ = TT.Vl;
vr_ = TT.Vr;

ax_ = TT.ax;
wz_ = TT.wz;



prevTimeStamp = time_(1)-0.005;
msg = struct('timestamp', 0,...
    'ticks_left', 0,...
    'ticks_right', 0,...
    'velocity_left', 0,...
    'velocity_right', 0,...
    'ax', 0,...
    'wz', 0,...
    'dt', 0);
estimated_pose = zeros(numel(time_), 4);
for i = 1:numel(time_)
    % if(sum(ismissing(TT(i,:))) > 0)
    %     continue;
    % end
    currentIndex = i;

    msg.timestamp = time_(currentIndex);
    msg.dt = msg.timestamp - prevTimeStamp;
    prevTimeStamp = time_(currentIndex);

    msg.ticks_left = ticksl_(currentIndex);
    msg.ticks_right = ticksr_(currentIndex);

    msg.velocity_left = vl_(currentIndex);
    msg.velocity_right = vr_(currentIndex);

    msg.ax = ax_(currentIndex);
    msg.wz = wz_(currentIndex);

    odom = update(odom, msg);

    estimated_pose(i,:) = pose(odom);
end
resTransformed = [0 1; -1 0] * [estimated_pose(:,1)' ; estimated_pose(:,2)'];
x = timeseries(resTransformed(1,:)', estimated_pose(:,4));
y = timeseries(resTransformed(2,:)', estimated_pose(:,4));
theta = timeseries(estimated_pose(:,3)', estimated_pose(:,4));

res.x = x;
res.y = y;

res.time = estimated_pose(:,4);
res.theta = theta;

end
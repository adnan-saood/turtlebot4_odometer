% %% Clear variables and restart workspace
clear;
%
proj_startup;

%% get messages

scans = getScans(bagFile);
imu = getIMU(bagFile);
vlr = getWheelVels(bagFile);
tickslr = getTicks(bagFile);
robotOdom = getOdoms(bagFile);
mouse = getMouse(bagFile);

TT = synchronize(scans, imu, vlr, tickslr, robotOdom);

N = numel(TT.Time);

iters = [];
for i = 1:N
    if(sum(ismissing(TT(i,:))) > 0)
        continue;
    end
    iters = [iters i];
end
TT = TT(iters,:);
%% Create Odometer Object

vars1 = 5;
vars1 = 10.^vars1;

vars2 = -5;
vars2 = 10.^vars2;

rmsResult = [];

fprintf("Index of %d: ", numel(vars1) * numel(vars2));

for ind1 = 1:numel(vars1)
    for ind2 = 1:numel(vars2)
        odom = tb4Odometer;

        % S = [x; y; theta; V; W];

        % z = [V_l; V_r; theta_imu; V; omega_z];

        Q = diag([1e-1 1e-1 1e-2 1e-1 1e-1]);
        R = diag([1e-1 1e-1 1e-1 1e-1 1e12 1e12 1e12]);

        odom = setQ(odom, Q);
        odom = setR(odom, R);

        poses = EstimatePoses(odom, TT);

        trivialEstimationX =  TT.Xlaser;
        trivialEstimationY =  TT.Ylaser;


        errorPoseX = poses.x.Data - trivialEstimationX;
        errorPoseY = poses.y.Data - trivialEstimationY;
        errorPoseXRMS = rms(errorPoseX);
        errorPoseYRMS = rms(errorPoseY);

        rmsResult(ind1, ind2) = errorPoseXRMS + errorPoseYRMS;
        fprintf("%d - ", ind1);

    end
    fprintf("\n")
end
% semilogx(rmsResult(:,1), rmsResult(:,2:end),'DisplayName','rmsResult')
%%
figure;
subplot(211);

plot(poses.y.Time,poses.x.Data, 'r', 'LineWidth',1.5);
hold on;
plot(poses.x.Time, poses.y.Data, 'g', 'LineWidth',1.5);
subplot(212)
plot(seconds(TT.Time), TT.odomX, 'r', 'LineWidth',1.5)
hold on
plot(seconds(TT.Time), TT.odomY, 'g', 'LineWidth',1.5)
plot(seconds(TT.Time), TT.Xlaser, 'r', 'LineWidth',1.5)
hold on
plot(seconds(TT.Time), TT.Ylaser, 'g', 'LineWidth',1.5)

%%
plot(TT.Time, TT.odomY);
hold on
plot(TT.Time, TT.odomX);
plot(TT.Time, cos(poses.theta.Data));
plot(TT.Time, cos(TT.odomTheta));

%%
plot(TT.Time, TT.Ylaser);
hold on
plot(TT.Time, TT.odomY)

plot(TT.Time, TT.ThetaLaser);
hold on
plot(TT.Time, TT.odomTheta)
%%
figure;
plot(poses.x.Data, poses.y.Data);
hold on
plot(trivialEstimationX, trivialEstimationY)
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
msgs = struct('timestamp', 0,...
    'ticks_left', 0,...
    'ticks_right', 0,...
    'velocity_left', 0,...
    'velocity_right', 0,...
    'laserX', 0, ...
    'laserY', 0, ...
    'laserTheta', 0, ...
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

    msg.velocity_right = vr_(currentIndex) * odom.r;
    msg.velocity_left = vl_(currentIndex) * odom.r;

    msg.ax = ax_(currentIndex);
    msg.wz = wz_(currentIndex);

    msg.laserX = TT.Xlaser(currentIndex);
    msg.laserY = TT.Ylaser(currentIndex);
    msg.laserTheta = TT.ThetaLaser(currentIndex);

    odom = update(odom, msg);

    estimated_pose(i,:) = pose(odom);
end


resTransformed = [estimated_pose(:,1)' ; estimated_pose(:,2)'];
x = timeseries(resTransformed(1,:)', estimated_pose(:,4));
y = timeseries(resTransformed(2,:)', estimated_pose(:,4));
theta = timeseries(estimated_pose(:, 3), estimated_pose(:,4));

res.x = x;
res.y = y;

res.time = estimated_pose(:,4);
res.theta = theta;

end
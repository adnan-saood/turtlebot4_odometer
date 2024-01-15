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

TT = synchronize(scans, imu, vlr, tickslr, robotOdom, mouse);

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

        Q = diag([1e-1 1e-1 5e-2 1e-1 1e-1]);
        R = diag([1e-1 1e-1 1e-1 1e-1 1e13 1e13 1e13]);
        
        % Q = diag([1e-1 1e-1 1e-2 1e-1 1e-1]);
        % R = diag([1e-1 1e-1 1e-1 1e-1]);

        odom = setQ(odom, Q);
        odom = setR(odom, R);

        [poses, errorsP] = EstimatePoses(odom, TT);

        trivialEstimationX =  TT.odomX;
        trivialEstimationY =  TT.odomY;


        errorPoseX = poses.x.Data - trivialEstimationX;
        errorPoseY = poses.y.Data - trivialEstimationY;
        errorPoseXRMS = rms(errorPoseX);
        errorPoseYRMS = rms(errorPoseY);

        rmsResult(ind1, ind2) = errorPoseXRMS + errorPoseYRMS;
        fprintf("%d - ", ind1);

    end
    fprintf("\n")
end
plot(seconds(TT.Time) - seconds(TT.Time(1)), errorsP);
legend(["X", "Y", "Theta", "V", "W"])
% semilogx(rmsResult(:,1), rmsResult(:,2:end),'DisplayName','rmsResult')
%%
figure;
subplot(311);
plot(poses.x.Time, poses.x.Data, "Color",[0.7 0 0], "LineWidth",1.2);
hold on;
plot(seconds(scans.Time(1:10:end)), scans.Xlaser(1:10:end), ...
    "Color",[0.7 0 0 0.1], ...
    "LineStyle","none", ...
    "LineWidth",0.1, ...
    "Marker","o", ...
    "MarkerEdgeColor",[0.7 0.4 0.4]);

subplot(312);
plot(poses.y.Time, poses.y.Data, "Color",[0 0.7 0], "LineWidth",1.2);
hold on;
plot(seconds(scans.Time(1:10:end)), scans.Ylaser(1:10:end), ...
    "Color",[0 0.7 0 0.1], ...
    "LineStyle","none", ...
    "LineWidth",0.1, ...
    "Marker","o", ...
    "MarkerEdgeColor",[0.4 0.7 0.4]);

subplot(313);
plot(poses.theta.Time, poses.theta.Data, "Color",[0 0 0.7], "LineWidth",1.2);
hold on;
plot(seconds(scans.Time(1:10:end)), scans.ThetaLaser(1:10:end), ...
    "Color",[0 0 0.7 0.1], ...
    "LineStyle","none", ...
    "LineWidth",0.1, ...
    "Marker","o", ...
    "MarkerEdgeColor",[0.4 0.4 0.7]);

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
function [res, estimated_errors] = EstimatePoses(odom, TT)

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
    P = getP(odom);
    estimated_errors(i,:) = diag(P)';
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
function out = getOdoms(bagFile)
odoms = read_bag(bagFile, "/odom");
n = numel(odoms);
X = zeros(n, 1);
Y = zeros(n, 1);
Theta = zeros(n, 1);
t = zeros(n, 1);
T_start = stamp2Sec(odoms{1}.header);
for i = 1:numel(odoms)
    X(i) = odoms{i}.pose.pose.position.x;
    Y(i) = odoms{i}.pose.pose.position.y;
    R = quat2eul([odoms{i}.pose.pose.orientation.w ...
        odoms{i}.pose.pose.orientation.x ...
        odoms{i}.pose.pose.orientation.y ...
        odoms{i}.pose.pose.orientation.z], "ZYX");
    Theta(i) = R(1);
    t(i) = stamp2Sec(odoms{i}.header);
end

initialOdom = [X(1) ; Y(1) ; Theta(1)];

% X = X - initialOdom(1);
% Y = Y - initialOdom(2);
% Theta = Theta - initialOdom(3);


tsc = tscollection({ ...
    timeseries(X, t, 'Name', 'odomX'), ...
    timeseries(Y, t, "Name", "odomY"), ...
    timeseries(Theta, t, "Name", "odomTheta")...
    });


tsc.odomX.DataInfo.Units = "m";
tsc.odomY.DataInfo.Units = "m";
tsc.odomTheta.DataInfo.Units = "rad";

tsc.odomX.DataInfo.Interpolation = "zoh";
tsc.odomY.DataInfo.Interpolation = "zoh";
tsc.odomTheta.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.odomX, tsc.odomY, tsc.odomTheta);

end


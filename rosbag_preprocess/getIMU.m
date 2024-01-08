function out = getIMU(bagFile)

IMU = read_bag(bagFile, "/imu");

vals.A.x = zeros(numel(IMU), 1);
vals.A.y = zeros(numel(IMU), 1);
vals.A.z = zeros(numel(IMU), 1);

vals.W.x = zeros(numel(IMU), 1);
vals.W.y = zeros(numel(IMU), 1);
vals.W.z = zeros(numel(IMU), 1);

t = zeros(numel(IMU), 1);

t_start = stamp2Sec(IMU{1}.header);
for i = 1:numel(IMU)

    vals.A.x(i) = IMU{i}.linear_acceleration.x;
    vals.A.y(i) = IMU{i}.linear_acceleration.y;
    vals.A.z(i) = IMU{i}.linear_acceleration.z;

    vals.W.x(i) = IMU{i}.angular_velocity.x;
    vals.W.y(i) = IMU{i}.angular_velocity.y;
    vals.W.z(i) = IMU{i}.angular_velocity.z;

    t(i) = stamp2Sec(IMU{i}.header);
end

tsc = tscollection({...
    timeseries(vals.A.x, t, 'Name', 'ax'), ...
    timeseries(vals.A.y, t, 'Name', 'ay'), ...
    timeseries(vals.A.z, t, 'Name', 'az'), ...
    timeseries(vals.W.x, t, 'Name', 'wx'), ...
    timeseries(vals.W.y, t, 'Name', 'wy'), ...
    timeseries(vals.W.z, t, 'Name', 'wz') ...
});

tsc.ax.DataInfo.Units = "m.s^{-2}";
tsc.ay.DataInfo.Units = "m.s^{-2}";
tsc.az.DataInfo.Units = "m.s^{-2}";

tsc.wx.DataInfo.Units = "rad.s^{-1}";
tsc.wy.DataInfo.Units = "rad.s^{-1}";
tsc.wz.DataInfo.Units = "rad.s^{-1}";

tsc.ax.DataInfo.Interpolation = "zoh";
tsc.ay.DataInfo.Interpolation = "zoh";
tsc.az.DataInfo.Interpolation = "zoh";

tsc.wx.DataInfo.Interpolation = "zoh";
tsc.wy.DataInfo.Interpolation = "zoh";
tsc.wz.DataInfo.Interpolation = "zoh";


out = timeseries2timetable(tsc.ax, tsc.ay, tsc.az, tsc.wx, tsc.wy, tsc.wz);


end


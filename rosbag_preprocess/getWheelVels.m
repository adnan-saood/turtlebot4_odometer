function out = getWheelVels(bagFile)

msgs = read_bag(bagFile, "/wheel_vels");
n = numel(msgs);
vl = zeros(n, 1);
vr = zeros(n,1);
t = zeros(n,1);

for i = 1:n
vl(i) = msgs{i}.velocity_left;
vr(i) = msgs{i}.velocity_right;
t(i) = stamp2Sec(msgs{i}.header);

end


tsc = tscollection({ ...
    timeseries(vl, t, 'Name', 'Vl'), ...
    timeseries(vr, t, "Name", "Vr"), ...
    });


tsc.Vl.DataInfo.Units = "rad.s^{-1}";
tsc.Vr.DataInfo.Units = "rad.s^{-1}";

tsc.Vl.DataInfo.Interpolation = "zoh";
tsc.Vr.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.Vl, tsc.Vr);

end


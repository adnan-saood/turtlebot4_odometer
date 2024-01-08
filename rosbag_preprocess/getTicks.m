function out = getTicks(bagFile)

msgs = read_bag(bagFile, "/wheel_ticks");
n = numel(msgs);
Tl = zeros(n, 1);
Tr = zeros(n,1);
t = zeros(n,1);

for i = 1:n
Tl(i) = msgs{i}.ticks_left;
Tr(i) = msgs{i}.ticks_right;
t(i) = stamp2Sec(msgs{i}.header);

end


tsc = tscollection({ ...
    timeseries(Tl, t, 'Name', 'TicksL'), ...
    timeseries(Tr, t, "Name", "TicksR"), ...
    });


tsc.TicksL.DataInfo.Units = "ticks";
tsc.TicksR.DataInfo.Units = "ticks";

tsc.TicksL.DataInfo.Interpolation = "zoh";
tsc.TicksR.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.TicksL, tsc.TicksR);


end


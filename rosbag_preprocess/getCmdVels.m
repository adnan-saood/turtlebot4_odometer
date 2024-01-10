function out = getCmdVels(bagFile)

% msgs = read_bag(bagFile, "/cmd_vel");
msgsStamp = read_bag(bagFile, "/wheel_vels");
n = numel(msgsStamp);
V = zeros(n, 1);
W = zeros(n,1);
t = zeros(numel(msgsStamp,1));

% for i = 1:n
% V(i) = msgs{i}.linear.x;
% W(i) = msgs{i}.angular.z;
% end

for m = 1:numel(msgsStamp)
    t(m) = stamp2Sec(msgsStamp{m}.header);
end


tsc = tscollection({ ...
    timeseries(0.1 * ones(size(t)), t, 'Name', 'cmdV'), ...
    timeseries(0.25 * ones(size(t)), t, "Name", "cmdW"), ...
    });


tsc.cmdV.DataInfo.Units = "m.s^{-1}";
tsc.cmdW.DataInfo.Units = "rad.s^{-1}";

tsc.cmdV.DataInfo.Interpolation = "zoh";
tsc.cmdW.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.cmdV, tsc.cmdW);

end


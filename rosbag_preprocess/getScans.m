function  out = getScans(bagFile)
% getScans - returns a list of scans in a struct

scans = read_bag(bagFile, '/scan');

orgPC = scan2PC(scans{1});

prevT = rigidtform3d(eye(3), [0 0 0]');

t_start = stamp2Sec(scans{1}.header);

x = zeros(numel(scans),1);
y = zeros(numel(scans),1);
theta = zeros(numel(scans),1);
t = zeros(numel(scans),1);

for i = 1:numel(scans)
    pt = scan2PC(scans{i});
    tform = pcregistericp(pt,orgPC, 'Metric','pointToPoint', 'InitialTransform',prevT);
    prevT = tform;
    x(i,1) = tform.Translation(1);
    y(i,1) = tform.Translation(2);
    R = rotm2axang(tform.R);
    theta(i,1) = R(4);
    t(i) = stamp2Sec(scans{i}.header);
end

tsc = tscollection({ ...
    timeseries(x, t, 'Name', 'Xlaser'), ...
    timeseries(y, t, "Name", "Ylaser"), ...
    timeseries(theta, t, "Name" , "ThetaLaser") ...
    });


tsc.Xlaser.DataInfo.Units = "m";
tsc.Ylaser.DataInfo.Units = "m";
tsc.ThetaLaser.DataInfo.Units = "rad";

tsc.Xlaser.DataInfo.Interpolation = "zoh";
tsc.Ylaser.DataInfo.Interpolation = "zoh";
tsc.ThetaLaser.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.Xlaser, tsc.Ylaser, tsc.ThetaLaser);

end


function  out = getScans(bagFile)
% getScans - returns a list of scans in a struct

scans = read_bag(bagFile, '/scan');


prevT = rigidtform3d(eye(3), [0 0 0]');

t_start = stamp2Sec(scans{1}.header);

x = zeros(numel(scans),1);
y = zeros(numel(scans),1);
rmse = zeros(numel(scans), 1);
theta = zeros(numel(scans),1);
t = zeros(numel(scans),1);

T_f = [rotz(90) , [-0.035; -0.01; 0.0] ; [0 0 0 1]];
T_inv = rigidtform3d((T_f));


orgPC = scan2PC(scans{1});
orgPC = pctransform(orgPC, T_inv);

for i = 1:numel(scans)
    pt = scan2PC(scans{i});
    pt = pctransform(pt,T_inv);
    [tform, pt, rmse_] = pcregisterndt(pt,orgPC,1.0,'verbose', false, 'InitialTransform',prevT);
    prevT = tform;
    x(i,1) = tform.Translation(1);
    y(i,1) = tform.Translation(2);
    rmse(i,1) = rmse_;
    R = rotm2eul(tform.R,"ZYX");
    theta(i,1) = R(1);
    % theta(i,1) = 2 * atan2(tform.R(2,1), tform.R(1,1));
    % if(R(3) < -0.9)
    %     theta(i,1) = 2*pi-theta(i,1);
    %     if(abs(R(3)) < 0.5)
    %         theta(i,1) = 0;
    %     end
    % end
    t(i) = stamp2Sec(scans{i}.header);
end

tsc = tscollection({ ...
    timeseries(x, t, 'Name', 'Xlaser'), ...
    timeseries(y, t, "Name", "Ylaser"), ...
    timeseries(theta, t, "Name" , "ThetaLaser") ...
    timeseries(rmse, t, "Name" , "RMSELaser") ...
    });


tsc.Xlaser.DataInfo.Units = "m";
tsc.Ylaser.DataInfo.Units = "m";
tsc.ThetaLaser.DataInfo.Units = "rad";
tsc.RMSELaser.DataInfo.Units = "m";

tsc.Xlaser.DataInfo.Interpolation = "zoh";
tsc.Ylaser.DataInfo.Interpolation = "zoh";
tsc.ThetaLaser.DataInfo.Interpolation = "zoh";
tsc.RMSELaser.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.Xlaser, tsc.Ylaser, tsc.ThetaLaser, tsc.RMSELaser);

end


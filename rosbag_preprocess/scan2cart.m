function [x,y,z] = scan2cart(in_scan)
r_ = in_scan.ranges;
thetas = linspace(in_scan.angle_min, in_scan.angle_max, 720)';
[x,y,z] = pol2cart(thetas,r_,zeros(size(r_)));
x = x(in_scan.intensities ~= 0);
y = y(in_scan.intensities ~= 0);
z = zeros(size(x));

end
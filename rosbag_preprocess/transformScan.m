function [x,y,z] = transformScan(Tin, in_scan)



[x1,y1,z1] = scan2cart(in_scan);

X = [x1' ; y1' ; z1'; ones(size(x1'))];
X = Tin.A * X;
x = X(1,:)';
y = X(2,:)';
z = X(3,:)';

end
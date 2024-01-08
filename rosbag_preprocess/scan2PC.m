function out = scan2PC(in_scan)
[x,y,z] = scan2cart(in_scan);

out = pointCloud([x y z]);
end

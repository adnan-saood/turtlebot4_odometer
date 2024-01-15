function T = poseToTransform(pose)
    if(size(pose,1) == 1)
        pose = pose';
    end
    if(size(pose,1) == 3)
        T = [cos(pose(3)) -sin(pose(3)) 0 pose(1);
             sin(pose(3))  cos(pose(3)) 0 pose(2);
             0             0            1 0;
             0             0            0 1];
    end
    if(size(pose,1) == 2)
        T = [1 0 pose(1);
             0 1 pose(2);
             0 0 1];
    end

end
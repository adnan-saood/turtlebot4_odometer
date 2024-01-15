proj_startup
scans = read_bag(bagFile, ["/scan"]);

%%

f = figure;
startIndex = 1;
stopIndex = 4000;

prevT = rigidtform3d(eye(3), [0 0 0]');
% -0.018605
T_f = [rotz(90) , [-0.035; -0.01; 0.0] ; [0 0 0 1]];
T_f2 = [rotz(90) , [0;0;0] ; [0 0 0 1]];
T_inv = rigidtform3d((T_f));

orgPC = scan2PC(scans{startIndex});
orgPC = pctransform(orgPC, T_inv);

for i = (startIndex+1):(min(stopIndex,numel(scans)))
    [x,y,z] = scan2cart(scans{i});
    X = T_f*[x';y';z';ones(size(x'))];
    pt = pointCloud((X(1:3,:))');
    % pt = pctransform(pt, T_inv);
    [tform, pt] = pcregisterndt(pt,orgPC,1.0,'verbose', false, 'InitialTransform',prevT);
    prevT = tform;
    T(i,:) = tform.Translation;
    % pcshowpair(pt,orgPC);
    % drawnow();
end
plot(T(:,1), T(:,2));
drawnow();
grid on;
grid minor
axis equal
[x_circle, y_circle, r_circle] = circleFit(T(:,1), T(:,2));
% viscircles([x_circle, y_circle], r_circle);
title(sprintf("Circle - Radius:%2.4f, Center: (%2.6f, %2.6f)", r_circle, x_circle, y_circle))
%% Plot all scans on top of each other

f2 = figure;

orgPC1 = scan2PC(scans{1});
pcshow(orgPC1);
hold on
T_f = [rotz(90) , [-0.040000; 0.000000; 0.00] ; [0 0 0 1]];
T_inv = rigidtform3d((T_f));
orgPc = pctransform(orgPC1, T_inv);
pcshowpair(orgPC1, orgPc)

%%
% plotScan(f2, scans{1});

prevT = rigidtform3d(eye(3), [0 0 0]');


for i = 750:numel(scans)
    pt = scan2PC(scans{i});
    pt = pctransform(pt, T_inv);
    [tform, pt] = pcregisterndt(pt,orgPC,1.0,'verbose', false, 'InitialTransform',prevT);
    prevT = tform;

    plotScan(f2, scans{1});
    hold on;
    [x2, y2, ~] = transformScan(tform, scans{i});
    plotScanCart(f2,x2,y2);
    drawnow();
    % hold off
end





%%



f = figure;
startIndex = 500;
M = [-0.04];
for s = 1:numel(M)
    
    prevT = rigidtform3d(eye(3), [0 0 0]');
    
    T_f = [rotz(90) , [-0.04; -0.010; 0.098715+0.094200] ; [0 0 0 1]];
    T_f2 = [rotz(90) , [0;0;0] ; [0 0 0 1]];
    T_inv = rigidtform3d(T_f);
    
    orgPC = scan2PC(scans{startIndex});
    orgPC = pctransform(orgPC, T_inv);
    
    for i = startIndex+1:numel(scans)
        pt = scan2PC(scans{i});
        pt = pctransform(pt, T_inv);
        [tform, pt] = pcregisterndt(pt,orgPC,1.0,'verbose', false, 'InitialTransform',prevT);
        prevT = tform;
        T(i,:) = tform.Translation;
        % pcshowpair(pt,orgPC);
        % drawnow();
    end
    plot(T(:,1), T(:,2));
    grid on;
    grid minor
    axis equal
    hold on;
    [x_circle, y_circle, r_circle] = circleFit(T(:,1), T(:,2));
    viscircles([x_circle, y_circle], r_circle);

    drawnow();
end

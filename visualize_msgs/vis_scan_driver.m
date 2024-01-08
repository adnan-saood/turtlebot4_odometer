
scans = read_bag(bagFile, ["/scan"]);

%%

f = figure;

orgPC = scan2PC(scans{1});

prevT = rigidtform3d(eye(3), [0 0 0]');

for i = 1:numel(scans)
    pt = scan2PC(scans{i});
    [tform, pt] = pcregistericp(pt,orgPC, 'Metric','pointToPoint', 'InitialTransform',prevT);
    prevT = tform;
    T(i,:) = tform.Translation;
    % pcshowpair(pt,orgPC);
    % drawnow();
end
plot(T(:,1), T(:,2));
drawnow();
hold on;
fprintf("k = %d\n", k);

%% Plot all scans on top of each other

f2 = figure;

orgPC = scan2PC(scans{1});
plotScan(f2, scans{1});

prevT = rigidtform3d(eye(3), [0 0 0]');

for i = 1:numel(scans)
    pt = scan2PC(scans{i});
    [tform, pt] = pcregistericp(pt,orgPC, 'Metric','pointToPoint', 'InitialTransform',prevT);
    prevT = tform;

    plotScan(f2, scans{1});
    hold on;
    [x2, y2, ~] = transformScan(tform, scans{i});
    plotScanCart(f2,x2,y2);
    drawnow();
    hold off
end





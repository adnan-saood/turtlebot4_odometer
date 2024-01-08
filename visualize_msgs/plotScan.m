function plotScan(parentFig, in_scan)
figure(parentFig);

[x,y,~] = scan2cart(in_scan);

scatter(x,y,'r.');
xlim([-4 4]);
ylim([-4 4]);
end




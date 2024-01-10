function out = getMouse(bagFile)

mouse = read_bag(bagFile, "/mouse");
n = numel(mouse);
mouseX = zeros(n, 1);
mouseY = zeros(n, 1);
t = zeros(n, 1);

for i = 1:numel(mouse)
    mouseX(i) = mouse{i}.integrated_x;
    mouseY(i) = mouse{i}.integrated_y;
    t(i) = stamp2Sec(mouse{i}.header);
end

Tmouse = [rotz(-45), [0; 0 ; 0]; [0 0 0 1]];

MouseCombined = [mouseX' ; mouseY'; zeros(size(mouseX')) ; ones(size(mouseX'))];

MouseCombined = inv(Tmouse) * MouseCombined;

mouseX = MouseCombined(1,:)';
mouseY = MouseCombined(2,:)';


tsc = tscollection({ ...
    timeseries(mouseX, t, 'Name', 'mouseX'), ...
    timeseries(mouseY, t, "Name", "mouseY") ...
    });


tsc.mouseX.DataInfo.Units = "m";
tsc.mouseY.DataInfo.Units = "m";

tsc.mouseX.DataInfo.Interpolation = "zoh";
tsc.mouseY.DataInfo.Interpolation = "zoh";

out = timeseries2timetable(tsc.mouseX, tsc.mouseY);

end


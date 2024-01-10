initPose = [0; 0 ; 0];
secondPose = [1 ; 1;  0.3];
global T_scale;
T_scale = 0.1;

T = poseToTransform(initPose);

f1 = figure;
hold on;


t = linspace(0,20,100);
for i = 1:numel(t)
    x = sin(2 * pi *t(i) / 20);
    y = cos(2 * pi *t(i) / 20 );
    theta = 0;
    p = [x;y;theta];
    T = poseToTransform(p);
    visFig(f1,T);



end

function T = poseToTransform(pose)
x = pose(1);
y = pose(2);
theta = pose(3);

T = [rotz(theta * pi/180.0) , [x y 0]';[0 0 0 1]];
end


function fig = visFig(figIn, T)
global T_scale;
axis equal
links = T_scale * [0 1 0 0;
                   0 0 1 0;
                   0 0 0 1];

linksT = T * [links; 1 1 1 1];

X0 = linksT(1,1);
X1 = linksT(1,2);

Y0 = linksT(2,1);
Y1 = linksT(2,2);

X2 = linksT(1,3);
Y2 = linksT(2,3);

plot([X0 X1], [Y0 Y1], 'Color','r','LineWidth',3);
hold on
plot([X0 X2], [Y0 Y2], 'Color','g','LineWidth',3);

scatter(X0, Y0, 'filled', 'MarkerFaceColor','b','SizeData',45);

end
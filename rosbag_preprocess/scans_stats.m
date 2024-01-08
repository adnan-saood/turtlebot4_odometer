S = getScans(bagFile);
%% Static histogram

preMoveIndex = 400;
postMoveIndec = 1720;

[xh1, yh1] = deal(S.x(1:preMoveIndex), S.y(1:preMoveIndex));
[xh1, yh1] = deal(xh1 - mean(xh1), yh1 - mean(yh1));

[xh2, yh2] = deal(S.x(postMoveIndec:end), S.y(postMoveIndec:end));
[xh2, yh2] = deal(xh2 - mean(xh2), yh2 - mean(yh2));

gr = categorical(size([xh1;xh2]));
gr(1:preMoveIndex) = 'pre';
gr(preMoveIndex+1:preMoveIndex+numel(yh2)) = 'post';


scatterhist([xh1;xh2], [yh1;yh2], 'NBins',30,'Marker','.', 'Group',gr)
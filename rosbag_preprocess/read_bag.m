function out = read_bag(bagPath, selectedToptics)
% Reads the bag easily
% bagPath: path to bag file
% selectedTopics: cell array of topic names. Empty returns all

reader = ros2bagreader(bagPath);

info = ros2("bag", "info", bagPath);

if(nargin == 1)
    out = readMessages(reader);
else
    out = readMessages(select(reader,"Topic",selectedToptics));
end 

end
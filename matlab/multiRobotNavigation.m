%% Navigating on a map with multiple robots

% Clear workspace
clear
delete(timerfindall)

% Set python version to 3.9
pyenv('Version','/usr/bin/python3.9');

% Reset ROS connection and main node
rosshutdown;
pause(1);
rosinit('NodeName','matlab/multiRobotNavigation');

% Set time variables
sampleTime = 0.05;
r = rateControl(1/sampleTime);

%% Create and publish map
map_server = mapServer;
% show(map.map.contents)

%% run forever
while true
end
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

% Vehicle amount
n = 2;

%% Create and publish map
map_server = mapServer;
% show(map.map.contents)

%% Create and run vehicles
vehicles = [];
for i=1:n
%     v = vehicle(sampleTime, map_server.map);
%     v.subscriber = ros.Subscriber(v.node, '/' + v.namespace + '/cmd_vel','geometry_msgs/Twist', @v.velocityCallback, 'DataFormat','struct');
    vehicles = [vehicles vehicle(sampleTime, map_server.map)];
end

% vehicles(1).drive;

%% Run until button click
mydlg = msgbox('Click the button to properly end the program.', 'Multi Robot Navigation');
waitfor(mydlg);

%% Delete ROS objects and timers, clean classes
% delete(map_server)
% delete(vehicles)
stop(timerfindall)
clear vehicle
rosshutdown
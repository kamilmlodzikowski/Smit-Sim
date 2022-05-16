%% Navigating on a map with multiple robots

% Clear workspace properly
try
    delete(vehicles)
catch
end
try
    delete(map_server)
catch
end
clear
clear vehicle
for t = timerfindall
    t.stop
end
clear t
delete(timerfindall)

% Set python version to 3.9
pyenv('Version','/usr/bin/python3.9');

% Reset ROS connection and main node
rosshutdown;
pause(1);
rosinit('NodeName','matlab/multiRobotNavigation');

%% Runtime variables
% Set time variable
sampleTime = 0.1;

% Vehicle amount
n = 10;

%% Create and publish map
map_server = mapServer;
% show(map.map.contents)

%% Create and run vehicles
vehicles = [];
for i=1:n
    vehicles = [vehicles vehicle(sampleTime, map_server.map)];
end
for i=1:n
    vehicles(i).start;
end

% vehicles(1).drive;

%% Run until button click
mydlg = msgbox('Click the button to properly end the program.', 'Multi Robot Navigation');
waitfor(mydlg);

%% Delete ROS objects and timers, clean classes
stop(timerfindall)
clear vehicle
rosshutdown
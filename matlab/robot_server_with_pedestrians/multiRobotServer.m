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
rosinit('NodeName','matlab/multiRobotServer');

% Enable multithreading
try
    parpool
catch
end
%% Runtime variables
% Set time variable
sampleTime = 0.05;

% Vehicle amount
n = 1;

% Pedestrian amount
p = 10;

%% Create and publish map
map_server = mapServer;
for i=1:p
    map_server.addPedestrian()
end
pedestrians = map_server.pedestrians;
show(map_server.map.contents)

%% Create and run vehicles
% vehicles = [];
% for i=1:n
%     vehicles = [vehicles vehicle(sampleTime, map_server.map)];
% end
% for i=1:n
%     vehicles(i).start;
% end

% vehicles(1).drive;

%% Run for set cycles
step = 0;
rate = rateControl(1/sampleTime);
tic
while (step < 200)
    map_server.walk(sampleTime);
    show(map_server.pedMap)
    step = step + 1;
    waitfor(rate);
end
toc

%% Run until button click
% mydlg = msgbox('Click the button to properly end the program.', 'Multi Robot Navigation');
% waitfor(mydlg);

%% Delete ROS objects and timers, clean classes
try
stop(timerfindall)
catch
end
clear vehicle
rosshutdown
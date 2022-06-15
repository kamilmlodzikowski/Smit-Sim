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
% try
%     parpool
% catch
% end
%% Runtime variables
% Set time variable
sampleTime = 0.05;

% Vehicle amount
n = 2;

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
vehicles = [];
for i=1:n
    vehicles = [vehicles vehicle(sampleTime, map_server.pedMap)];
end

%% Run for set cycles
step = 0;
stepN = 2000;
rate = rateControl(1/sampleTime);
tic
while (step < stepN)
    map_server.walk(sampleTime);
    for i=1:n
        vehicles(i).drive();
        scan = lidarScan(vehicles(i).ranges, vehicles(i).angles);
        validScan = removeInvalidData(scan,'RangeLimits',[0,vehicles(i).lidar.Range(2)]);
        insertRay(map_server.pedMap,vehicles(i).pose',validScan,vehicles(i).lidar.Range(2));
    end
    hold off
    show(map_server.pedMap)
    hold on
    for i=1:n
        plotTrvec = [vehicles(i).pose(1:2); 0];
        plotRot = axang2quat([0 0 1 vehicles(i).pose(3)]);
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1);
    end
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
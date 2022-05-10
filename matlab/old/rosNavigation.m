%% Testing ROS connection
clear
close all
matlab_visualization = false;
% setting python version to 3.9
pyenv('Version','/usr/bin/python3.9');
% initialize ROS connection
rosshutdown
rosinit('NodeName','/matlab');

%% Time variables
sampleTime = 0.05;
r = rateControl(1/sampleTime);

%% Initialize map
global map
map = createRandomMap([50 50], 10, 78, 5, 100, {'Box','Plus','Circle'});
if matlab_visualization
    globalFig = figure('Name','Global');
    show(map);
    lidarMap = binaryOccupancyMap(50,50,10);
    localFig = figure('Name','Local');
    show(lidarMap);
end

%% Vehicle and sensor init
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
pose = [2;2;0];
lidar = rangeSensor;
lidar.Range = [0,10];

%% Create ROS interfaces
% map server
mapServer = rossvcserver("/static_map","nav_msgs/GetMap",@getMapCallback,"DataFormat","struct");
% cmd_vel subscriber
global linVel angVel
linVel = 0;
angVel = 0;
velSub = rossubscriber('/cmd_vel','geometry_msgs/Twist', @velocityCallback, 'DataFormat','struct');
% lidar publisher
lidarPub = rospublisher('/scan','sensor_msgs/LaserScan',"DataFormat","struct");
lidarMsg = rosmessage(lidarPub);
lidarMsg.Header.FrameId = 'laser_scan';
lidarMsg.Header.Stamp = rostime('now','DataFormat','struct');
lidarMsg.TimeIncrement = single(sampleTime/258);
lidarMsg.AngleMin = single(lidar.HorizontalAngle(1));
lidarMsg.AngleMax = single(lidar.HorizontalAngle(2));
lidarMsg.AngleIncrement = single(lidar.HorizontalAngleResolution);
lidarMsg.RangeMin = single(0);
lidarMsg.RangeMax = single(lidar.Range(2));
% lidarMsg.ScanTime = single(sampleTime);
% odom publisher
odomPub = rospublisher('/odom','nav_msgs/Odometry','DataFormat','struct');
odomMsg = rosmessage(odomPub);
odomMsg.Header.FrameId = 'odom';
odomMsg.ChildFrameId = 'base_link';
odomMsg.Pose.Pose.Position.X = pose(1);
odomMsg.Pose.Pose.Position.Y = pose(2);
plotRot = axang2quat([0 0 1 pose(3)]);
odomMsg.Pose.Pose.Orientation.W = plotRot(1);
odomMsg.Pose.Pose.Orientation.Z = plotRot(4);
% odomTimer = timer;
% set(odomTimer, 'executionMode','fixedRate');
% set(odomTimer,'TimerFcn','send(odomPub, odomMsg)');
% set(odomTimer, 'Period', sampleTime);
% start(odomTimer);
% map publisher
mapPub = rospublisher('/map', 'nav_msgs/OccupancyGrid','DataFormat','struct');
mapMsg = rosmessage(mapPub);
mapMsg = rosWriteBinaryOccupancyGrid(mapMsg, map);
mapMsg.Header.FrameId = 'map';
pause(1);
send(mapPub,mapMsg);

%% TF
tftree = rostf;
% map to odom tf
tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = 'odom';
tfStampedMsg.Header.FrameId = 'map';
tfStampedMsg.Header.Stamp = rostime('now');
tfStampedMsg.Transform.Rotation.W = 1;
tfList = [tfStampedMsg];
% odom to base_link tf
tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = 'base_link';
tfStampedMsg.Header.FrameId = 'odom';
tfStampedMsg.Header.Stamp = rostime('now');
tfStampedMsg.Transform.Translation.X = pose(1);
tfStampedMsg.Transform.Translation.Y = pose(2);
tfStampedMsg.Transform.Rotation.W = plotRot(1);
tfStampedMsg.Transform.Rotation.Z = plotRot(4);
tfList = [tfList tfStampedMsg];
% base_link to laser_scan tf
tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = 'laser_scan';
tfStampedMsg.Header.FrameId = 'base_link';
tfStampedMsg.Header.Stamp = rostime('now');
tfStampedMsg.Transform.Translation.X = 0;
tfStampedMsg.Transform.Translation.Y = 0;
tfStampedMsg.Transform.Rotation.Z = 0;
tfStampedMsg.Transform.Rotation.W = 1;
tfList = [tfList tfStampedMsg];
for msg = tfList
    sendTransform(tftree, msg);
end

%% Control loop
if matlab_visualization
    ax1 = globalFig.CurrentAxes;
    ax2 = localFig.CurrentAxes;
    plotTrvec = [pose(1:2); 0];
    plotRot = axang2quat([0 0 1 pose(3)]);
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax1);
    figure(localFig)
end
while true

    % move robot
    vel = derivative(diffDrive, pose, [linVel angVel]);
    pose = pose + vel*sampleTime; 

    % Update visualization
    plotTrvec = [pose(1:2); 0];
    plotRot = axang2quat([0 0 1 pose(3)]);

    % send odometry message
    odomMsg.Pose.Pose.Position.X = pose(1);
    odomMsg.Pose.Pose.Position.Y = pose(2);
    odomMsg.Pose.Pose.Orientation.W = plotRot(1);
    odomMsg.Pose.Pose.Orientation.Z = plotRot(4);
    odomMsg.Twist.Twist.Linear.X = linVel;
    odomMsg.Twist.Twist.Angular.Z = angVel;
    send(odomPub,odomMsg);

    % send tfs
    tfTime = rostime('now');
    for msg = tfList
        if strcmp(msg.Header.FrameId,'odom')
            msg.Transform.Translation.X = pose(1);
            msg.Transform.Translation.Y = pose(2);
            msg.Transform.Rotation.W = plotRot(1);
            msg.Transform.Rotation.Z = plotRot(4);
        end
        msg.Header.Stamp = tfTime;
        sendTransform(tftree, msg);
    end

    % collect lidar data
    [ranges, angles] = lidar(pose', map);
    scan = lidarScan(ranges,angles);
    validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
    if matlab_visualization
        insertRay(lidarMap,pose',validScan,lidar.Range(2));
        show(lidarMap);
    end

    % send lidar message
    lidarMsg.Header.Stamp = rostime('now','DataFormat','struct');
    lidarMsg.Ranges = single(ranges);
    send(lidarPub, lidarMsg);
    
    if matlab_visualization
        % Delete image of the last robot to prevent displaying multiple robots
        items = get(ax1, 'Children');
        delete(items(1));

        % Plot robot onto known map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax1);
        % Plot robot on new map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax2);
    end

    waitfor(r);
end

%% Close ROS node
rosshutdown

%% Callback for map service
function resp = getMapCallback(~,~,resp)
    global map
    msg = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
    msg = rosWriteBinaryOccupancyGrid(msg, map);
    msg.Header.FrameId = 'map';
    resp.Map = msg;
    disp("Sending map...");
end

%% Callback for cmd_vel subscriber
function  velocityCallback(~, message)
    global linVel angVel
    linVel = message.Linear.X;
    angVel = message.Angular.Z;
end
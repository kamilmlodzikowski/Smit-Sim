clear all
close all
figure

% Map parameters
mapSize = [50 30];
resolution = 10;

% Maze generation
passageWidth = 78; % 94
wallThickness = 5;

map = mapMaze(passageWidth,wallThickness,"MapSize",mapSize,"MapResolution",resolution);

grid1 = map.occupancyMatrix;
grid2 = map.occupancyMatrix;
% Lower side walls size
for i = (wallThickness+1):(mapSize(1)*resolution-wallThickness)
    if sum(grid1(:,i)) == max(size(grid1(:,i)))
        grid1((wallThickness+1):(mapSize(2)*resolution-wallThickness),i) = 0;
    end
end
for i = (wallThickness+1):(mapSize(2)*resolution-wallThickness)
    if sum(grid2(i,:)) == max(size(grid2(i,:)))
        grid2(i,(wallThickness+1):(mapSize(1)*resolution-wallThickness)) = 0;
    end
end
map = binaryOccupancyMap(min(grid1,grid2), resolution);

mapInflated = copy(map);
inflate(mapInflated,0.25);

prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 250;
prm.ConnectionDistance = 10;
goalRadius = 0.3;

robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");

pedN = 10;
peds = [];

for i=1:pedN
    ped.initPose = [mapSize(1)*rand() mapSize(2)*rand()];
    while(getOccupancy(mapInflated, ped.initPose))
        ped.initPose = [mapSize(1)*rand() mapSize(2)*rand()];
    end
    ped.goal = [mapSize(1)*rand() mapSize(2)*rand()];
    while (getOccupancy(mapInflated, ped.goal))
        ped.goal = [mapSize(1)*rand() mapSize(2)*rand()];
    end
    ped.currPose = [ped.initPose 0]';
    ped.path = findpath(prm, ped.initPose, ped.goal);
    ped.controller = controllerPurePursuit;
    ped.controller.Waypoints = ped.path;
    ped.controller.DesiredLinearVelocity = 1.4;
    ped.controller.MaxAngularVelocity = 3.14;
    ped.controller.LookaheadDistance = 0.3;
    peds = [peds ped];
end

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

while(true)
    pedMap = copy(map);
    for i=1:pedN
        % Re-compute the distance to the goal
        distanceToGoal = norm(peds(i).currPose(1:2) - peds(i).goal(:));

        if ( distanceToGoal > goalRadius )

            % Compute the controller outputs, i.e., the inputs to the robot
            [v, omega] = peds(i).controller(peds(i).currPose);

            % Get the robot's velocity using controller inputs
            vel = derivative(robot, peds(i).currPose, [v omega]);

            % Update the current pose
            peds(i).currPose = peds(i).currPose + vel*sampleTime;

            % Plot the path of the robot as a set of transforms
            plotTrVec = [peds(i).currPose(1:2); 0];
            plotRot = axang2quat([0 0 1 peds(i).currPose(3)]);
%             plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", robot.TrackWidth);
        else
            tmp = peds(i).initPose;
            peds(i).initPose = peds(i).goal;
            peds(i).goal = tmp;
            peds(i).path = flipud(peds(i).path);
            peds(i).controller.Waypoints = peds(i).path;
        end
        pedMap.setOccupancy([peds(i).currPose(1:2)'+0.1*[cos(peds(i).currPose(3)-pi/2) sin(peds(i).currPose(3)-pi/2)];peds(i).currPose(1:2)'-0.1*[cos(peds(i).currPose(3)-pi/2) sin(peds(i).currPose(3)-pi/2)]], 1);
    end
    hold off
    show(pedMap)
    hold all
    for i=1:pedN
        % Plot path each instance so that it stays persistent while robot mesh
        % moves
        plot(peds(i).path(:,1), peds(i).path(:,2),"g--d")
    end
    waitfor(vizRate);
end
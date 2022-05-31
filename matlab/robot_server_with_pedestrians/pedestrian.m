classdef pedestrian < handle

    properties
        goalPoseRadius

        robot
        controller

        initPose
        goalPose
        currPose
    end

    methods
        function obj = pedestrian(map)
            mapSize = fliplr(map.GridSize)./map.Resolution;
            mapInflated = copy(map);
            inflate(mapInflated,0.25);

            prm = mobileRobotPRM;
            prm.Map = mapInflated;
            prm.NumNodes = 250;
            prm.ConnectionDistance = 10;
            obj.goalPoseRadius = 0.3;
            obj.robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
            obj.initPose = [mapSize(1)*rand() mapSize(2)*rand()];
            while(getOccupancy(mapInflated, obj.initPose))
                obj.initPose = [mapSize(1)*rand() mapSize(2)*rand()];
            end
            obj.goalPose = [mapSize(1)*rand() mapSize(2)*rand()];
            while (getOccupancy(mapInflated, obj.goalPose))
                obj.goalPose = [mapSize(1)*rand() mapSize(2)*rand()];
            end
            obj.currPose = [obj.initPose 0]';
            path = findpath(prm, obj.initPose, obj.goalPose);
            obj.controller = controllerPurePursuit;
            obj.controller.Waypoints = path;
            obj.controller.DesiredLinearVelocity = 1.4;
            obj.controller.MaxAngularVelocity = 3.14;
            obj.controller.LookaheadDistance = 0.3;
        end

        function walk(obj, sampleTime)
            distanceToGoal = norm(obj.currPose(1:2) - obj.goalPose(:));

            if ( distanceToGoal > obj.goalPoseRadius )

                % Compute the controller outputs, i.e., the inputs to the robot
                [v, omega] = obj.controller(obj.currPose);

                % Get the robot's velocity using controller inputs
                vel = derivative(obj.robot, obj.currPose, [v omega]);

                % Update the current pose
                obj.currPose = obj.currPose + vel*sampleTime;
            else
                tmp = obj.initPose;
                obj.initPose = obj.goalPose;
                obj.goalPose = tmp;
                obj.controller.Waypoints = flipud(obj.controller.Waypoints);
            end
        end
    end
end
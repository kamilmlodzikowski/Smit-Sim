classdef vehicle < handle

    properties
        diffDrive

        lidar
        map

        pose
        velocity

        ranges
        angles

        namespace
        node
        subscriber
%         odomPublisher
%         odomMsg
%         lidarPublisher
%         lidarMsg
%         tfPublisher
%         tfMsg
%         timer

        sampleTime
    end

    methods
        function obj = vehicle(sampleTime, map)
            obj.diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

            obj.lidar = rangeSensor;
            obj.lidar.Range = [0,10];
            obj.map = map;

            obj.findRandomStartingPosition();
            obj.velocity = [0 0];

            persistent id
            if isempty(id)
                id = 1;
            else
                id = id + 1;
            end

            obj.namespace = 'vehicle_' + string(id);

            obj.node = ros.Node('matlab/'+obj.namespace);

            obj.subscriber = ros.Subscriber(obj.node, '/' + obj.namespace + '/cmd_vel','geometry_msgs/Twist', @obj.velocityCallback, 'DataFormat','struct');

%             obj.odomPublisher = ros.Publisher(obj.node,'/' + obj.namespace + '/odom','nav_msgs/Odometry','DataFormat','struct');

%             obj.odomMsg = rosmessage(obj.odomPublisher);
%             obj.odomMsg.Header.FrameId = 'odom';
%             obj.odomMsg.Header.Stamp = rostime('now','DataFormat','struct');
%             obj.odomMsg.ChildFrameId = [char(obj.namespace) '/base_link'];
%             obj.odomMsg.Pose.Pose.Position.X = obj.pose(1);
%             obj.odomMsg.Pose.Pose.Position.Y = obj.pose(2);
%             plotRot = axang2quat([0 0 1 obj.pose(3)]);
%             obj.odomMsg.Pose.Pose.Orientation.W = plotRot(1);
%             obj.odomMsg.Pose.Pose.Orientation.Z = plotRot(4);
%             send(obj.odomPublisher,obj.odomMsg);

%             obj.lidarPublisher = ros.Publisher(obj.node,'/' + obj.namespace + '/scan','sensor_msgs/LaserScan',"DataFormat","struct");

            [obj.ranges, obj.angles] = obj.lidar(obj.pose', copy(obj.map));
%             obj.lidarMsg = rosmessage(obj.lidarPublisher);
%             obj.lidarMsg.Header.FrameId = [char(obj.namespace) '/laser_scan'];
%             obj.lidarMsg.Header.Stamp = rostime('now','DataFormat','struct');
%             obj.lidarMsg.TimeIncrement = single(sampleTime/258);
%             obj.lidarMsg.AngleMin = single(obj.lidar.HorizontalAngle(1));
%             obj.lidarMsg.AngleMax = single(obj.lidar.HorizontalAngle(2));
%             obj.lidarMsg.AngleIncrement = single(obj.lidar.HorizontalAngleResolution);
%             obj.lidarMsg.RangeMin = single(0);
%             obj.lidarMsg.RangeMax = single(obj.lidar.Range(2));
%             obj.lidarMsg.Ranges = single(ranges);

%             obj.tfPublisher = ros.Publisher(obj.node, '/tf', 'tf2_msgs/TFMessage', 'DataFormat','struct');

%             obj.tfMsg = rosmessage(obj.tfPublisher, 'DataFormat','struct');
%             tfStampedMsg = rosmessage('geometry_msgs/TransformStamped', 'DataFormat','struct');
%             tfStampedMsg.ChildFrameId = [char(obj.namespace) '/laser_scan'];
%             tfStampedMsg.Header.FrameId = [char(obj.namespace) '/base_link'];
%             tfStampedMsg.Header.Stamp = rostime('now','DataFormat','struct');
%             tfStampedMsg.Transform.Translation.X = 0;
%             tfStampedMsg.Transform.Translation.Y = 0;
%             tfStampedMsg.Transform.Rotation.Z = 0;
%             tfStampedMsg.Transform.Rotation.W = 1;
%             obj.tfMsg.Transforms = tfStampedMsg;
%             send(obj.tfPublisher, obj.tfMsg);

%             plotRot = axang2quat([0 0 1 obj.pose(3)]);
%             tfStampedMsg.ChildFrameId = [char(obj.namespace) '/base_link'];
%             tfStampedMsg.Header.FrameId = 'odom';
%             tfStampedMsg.Transform.Translation.X = obj.pose(1);
%             tfStampedMsg.Transform.Translation.Y = obj.pose(2);
%             tfStampedMsg.Transform.Rotation.W = plotRot(1);
%             tfStampedMsg.Transform.Rotation.Z = plotRot(4);
%             obj.tfMsg.Transforms = [obj.tfMsg.Transforms tfStampedMsg];
%             send(obj.tfPublisher, obj.tfMsg);

            obj.sampleTime = sampleTime;

%             obj.timer = timer('Name',obj.namespace);
%             set(obj.timer,'executionMode','fixedRate');
%             set(obj.timer,'TimerFcn',@(~,~)obj.drive);
%             set(obj.timer,'Period',sampleTime);
        end

%         function start(obj)
%             startat(obj.timer, datetime + seconds(1));
%         end

        function findRandomStartingPosition(obj)
            while(true)
                obj.pose = [rand*obj.map.GridSize(2);rand*obj.map.GridSize(1);0];
                if checkOccupancy(obj.map, obj.pose(1:2)-obj.map.Resolution/2, ones(1,2)*obj.map.Resolution)
                    continue
                end
                break
            end
%             s = map.mapSize;
%             res = map.resolution;
%             while(true)
%                 obj.pose = [rand*s(1);rand*s(2);0];
%                 for x=obj.pose(1)*res-res/2:obj.pose(1)*res+res/2
%                     for y=obj.pose(2)*res-res/2:obj.pose(2)*res+res/2
%                         if checkOccupancy(map.contents,[x y])
%                             continue
%                         end
%                     end
%                 end
%                 break
%             end
        end

        function velocityCallback(obj,~,message)
            obj.velocity = [message.Linear.X message.Angular.Z];
        end

        function drive(obj)
            obj.pose = obj.pose + derivative(obj.diffDrive, obj.pose, obj.velocity)*obj.sampleTime;
            [obj.ranges, obj.angles] = obj.lidar(obj.pose', copy(obj.map));

%             obj.lidarMsg.Header.Stamp = rostime('now','DataFormat','struct');
%             obj.lidarMsg.Ranges = single(ranges);
%             send(obj.lidarPublisher, obj.lidarMsg);

%             obj.odomMsg.Header.Stamp = obj.lidarMsg.Header.Stamp;
%             obj.odomMsg.Pose.Pose.Position.X = obj.pose(1);
%             obj.odomMsg.Pose.Pose.Position.Y = obj.pose(2);
%             plotRot = axang2quat([0 0 1 obj.pose(3)]);
%             obj.odomMsg.Pose.Pose.Orientation.W = plotRot(1);
%             obj.odomMsg.Pose.Pose.Orientation.Z = plotRot(4);
%             obj.odomMsg.Twist.Twist.Linear.X = obj.velocity(1);
%             obj.odomMsg.Twist.Twist.Angular.Z = obj.velocity(2);
%             send(obj.odomPublisher,obj.odomMsg);

%             obj.tfMsg.Transforms(1).Header.Stamp = obj.lidarMsg.Header.Stamp;
%             obj.tfMsg.Transforms(2).Header.Stamp = obj.lidarMsg.Header.Stamp;
%             obj.tfMsg.Transforms(2).Transform.Translation.X = obj.pose(1);
%             obj.tfMsg.Transforms(2).Transform.Translation.Y = obj.pose(2);
%             obj.tfMsg.Transforms(2).Transform.Rotation.W = plotRot(1);
%             obj.tfMsg.Transforms(2).Transform.Rotation.Z = plotRot(4);
%             send(obj.tfPublisher, obj.tfMsg);
        end

        function delete(obj)
%             stop(obj.timer)
%             delete(obj.timer)
%             delete(obj.odomPublisher)
%             delete(obj.lidarPublisher)
            delete(obj.subscriber)
            delete(obj.node)
        end
    end
end
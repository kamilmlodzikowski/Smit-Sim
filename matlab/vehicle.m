classdef vehicle < handle

    properties
        diffDrive

        lidar
        map

        pose
        velocity

        namespace
        node
        subscriber
        odomPublisher
        lidarPublisher
        tftree
        timer

        sampleTime
    end

    methods
        function obj = vehicle(sampleTime, map)
            obj.diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");

            obj.lidar = rangeSensor;
            obj.lidar.Range = [0,10];
            obj.map = map;

            obj = obj.findRandomStartingPosition(map);
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

            obj.odomPublisher = ros.Publisher(obj.node,'/' + obj.namespace + '/odom','nav_msgs/Odometry','DataFormat','struct');
            obj.lidarPublisher = ros.Publisher(obj.node,'/' + obj.namespace + '/scan','sensor_msgs/LaserScan',"DataFormat","struct");
            obj.tftree = ros.TransformationTree(obj.node);

            obj.sampleTime = sampleTime;

            obj.timer = timer('Name',obj.namespace);
            set(obj.timer,'executionMode','fixedRate');
            set(obj.timer,'TimerFcn',@(~,~)obj.drive);
            set(obj.timer,'Period',sampleTime);
%             start(obj.timer);
        end

        function start(obj)
            startat(obj.timer, datetime + seconds(5));
        end

        function obj = findRandomStartingPosition(obj, map)
            s = map.mapSize;
            res = map.resolution;
            while(true)
                obj.pose = [rand*s(1);rand*s(2);0];
                for x=obj.pose(1)*res-res/2:obj.pose(1)*res+res/2
                    for y=obj.pose(2)*res-res/2:obj.pose(2)*res+res/2
                        if checkOccupancy(map.contents,[x y])
                            continue
                        end
                    end
                end
                break
            end
        end

        function velocityCallback(obj,~,message)
            obj.velocity = [message.Linear.X message.Angular.Z];
        end

        function drive(obj)
            obj.pose = obj.pose + derivative(obj.diffDrive, obj.pose, obj.velocity)*obj.sampleTime;
            [ranges, ~] = obj.lidar(obj.pose', obj.map.contents);

            lidarMsg = rosmessage(obj.lidarPublisher);
            lidarMsg.Header.FrameId = [char(obj.namespace) '/laser_scan'];
            lidarMsg.Header.Stamp = rostime('now','DataFormat','struct');
            lidarMsg.TimeIncrement = single(obj.sampleTime/258);
            lidarMsg.AngleMin = single(obj.lidar.HorizontalAngle(1));
            lidarMsg.AngleMax = single(obj.lidar.HorizontalAngle(2));
            lidarMsg.AngleIncrement = single(obj.lidar.HorizontalAngleResolution);
            lidarMsg.RangeMin = single(0);
            lidarMsg.RangeMax = single(obj.lidar.Range(2));
            lidarMsg.Ranges = single(ranges);
            send(obj.lidarPublisher, lidarMsg);

            odomMsg = rosmessage(obj.odomPublisher);
            odomMsg.Header.FrameId = 'odom';
            odomMsg.Header.Stamp = rostime('now','DataFormat','struct');
            odomMsg.ChildFrameId = [char(obj.namespace) '/base_link'];
            odomMsg.Pose.Pose.Position.X = obj.pose(1);
            odomMsg.Pose.Pose.Position.Y = obj.pose(2);
            plotRot = axang2quat([0 0 1 obj.pose(3)]);
            odomMsg.Pose.Pose.Orientation.W = plotRot(1);
            odomMsg.Pose.Pose.Orientation.Z = plotRot(4);
            odomMsg.Twist.Twist.Linear.X = obj.velocity(1);
            odomMsg.Twist.Twist.Angular.Z = obj.velocity(2);
            send(obj.odomPublisher,odomMsg);

            tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
            tfStampedMsg.ChildFrameId = [char(obj.namespace) '/base_link'];
            tfStampedMsg.Header.FrameId = 'odom';
            tfStampedMsg.Header.Stamp = rostime('now');
            tfStampedMsg.Transform.Translation.X = obj.pose(1);
            tfStampedMsg.Transform.Translation.Y = obj.pose(2);
            tfStampedMsg.Transform.Rotation.W = plotRot(1);
            tfStampedMsg.Transform.Rotation.Z = plotRot(4);
            sendTransform(obj.tftree, tfStampedMsg);

            tfStampedMsg.ChildFrameId = [char(obj.namespace) '/laser_scan'];
            tfStampedMsg.Header.FrameId = [char(obj.namespace) '/base_link'];
            tfStampedMsg.Header.Stamp = rostime('now');
            tfStampedMsg.Transform.Translation.X = 0;
            tfStampedMsg.Transform.Translation.Y = 0;
            tfStampedMsg.Transform.Rotation.Z = 0;
            tfStampedMsg.Transform.Rotation.W = 1;
            sendTransform(obj.tftree, tfStampedMsg);
        end

        function delete(obj)
            stop(obj.timer)
            delete(obj.timer)
            delete(obj.odomPublisher)
            delete(obj.lidarPublisher)
            delete(obj.subscriber)
            delete(obj.node)
        end
    end
end
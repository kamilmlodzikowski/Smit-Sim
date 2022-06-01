classdef mapServer < handle

    properties
        node
        server
        timer
        mapPublisher
        mapMsg
        tfPublisher
        tfMsg
        respMap
        map
        pedMap
        pedestrians
    end

    methods
        function obj = mapServer()
            obj.map = randomMap([50 30], 10, 78, 5, 10, {'Box','Plus','Circle'});
            obj.pedMap = copy(obj.map.contents);
            obj.pedestrians = [];

            obj.node = ros.Node('matlab/map');

            obj.server = ros.ServiceServer(obj.node,"/static_map","nav_msgs/GetMap",@(~,~,~)obj.getMapCallback,"DataFormat","struct");

            obj.respMap = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
            obj.respMap.Header.FrameId = 'map';
            obj.respMap.Header.Stamp = rostime('now','DataFormat','struct');
            obj.respMap = rosWriteBinaryOccupancyGrid(obj.respMap, obj.map.contents);
            
            obj.mapPublisher = ros.Publisher(obj.node,'/map', 'nav_msgs/OccupancyGrid','DataFormat','struct');

            obj.mapMsg = rosmessage(obj.mapPublisher);
            obj.mapMsg.Header.FrameId = 'map';
            obj.mapMsg.Header.Stamp = rostime('now','DataFormat','struct');
            obj.mapMsg = rosWriteBinaryOccupancyGrid(obj.mapMsg, obj.map.contents);
            send(obj.mapPublisher, obj.mapMsg);

            obj.tfPublisher = ros.Publisher(obj.node, '/tf', 'tf2_msgs/TFMessage', 'DataFormat','struct');

            obj.tfMsg = rosmessage(obj.tfPublisher, 'DataFormat','struct');
            tfStampedMsg = rosmessage('geometry_msgs/TransformStamped', 'DataFormat','struct');
            tfStampedMsg.ChildFrameId = 'odom';
            tfStampedMsg.Header.FrameId = 'map';
            tfStampedMsg.Header.Stamp = rostime('now', 'DataFormat','struct');
            tfStampedMsg.Transform.Translation.X = 0;
            tfStampedMsg.Transform.Translation.Y = 0;
            tfStampedMsg.Transform.Rotation.Z = 0;
            tfStampedMsg.Transform.Rotation.W = 1;
            obj.tfMsg.Transforms = tfStampedMsg;
            send(obj.tfPublisher, obj.tfMsg);

            obj.timer = timer('Name','MapPublisher');
            set(obj.timer,'executionMode','fixedRate');
            set(obj.timer,'TimerFcn',@(~,~)obj.sendMessage);
            set(obj.timer,'Period',0.05);
            start(obj.timer);
        end

        function addPedestrian(obj)
            obj.pedestrians = [obj.pedestrians pedestrian(obj.map.contents)];
            obj.updatePedMap()
        end

        function walk(obj, sampleTime)
            for i=1:size(obj.pedestrians ,2)
                obj.pedestrians(i).walk(sampleTime);
            end
            obj.updatePedMap()
        end

        function updatePedMap(obj)
            obj.pedMap = copy(obj.map.contents);
            for i=1:size(obj.pedestrians ,2)
                obj.pedMap.setOccupancy( ...
                    [obj.pedestrians(i).currPose(1:2)'+0.1*[cos(obj.pedestrians(i).currPose(3)-pi/2) sin(obj.pedestrians(i).currPose(3)-pi/2)]; ...
                    obj.pedestrians(i).currPose(1:2)'-0.1*[cos(obj.pedestrians(i).currPose(3)-pi/2) sin(obj.pedestrians(i).currPose(3)-pi/2)]], 1);
            end
        end

        function sendMessage(obj)
            t = rostime('now','DataFormat','struct');
            obj.mapMsg.Header.Stamp = t;
            send(obj.mapPublisher, obj.mapMsg);
            obj.tfMsg.Transforms.Header.Stamp = t;
            send(obj.tfPublisher, obj.tfMsg);
        end

        function resp = getMapCallback(obj,~,~,resp)
            obj.respMap.Header.Stamp = rostime('now','DataFormat','struct');
            resp.Map = obj.respMap;
        end

        function delete(obj)
            stop(obj.timer)
            delete(obj.timer)
            delete(obj.tfPublisher)
            delete(obj.mapPublisher)
            delete(obj.server)
            delete(obj.node)
        end
    end
end
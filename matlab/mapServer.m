classdef mapServer

    properties
        node
        timer
        mapPublisher
        mapMsg
        tfPublisher
        tfMsg
        server
        respMap
        map
    end

    methods
        function obj = mapServer()
            obj.map = randomMap([50 50], 10, 78, 5, 100, {'Box','Plus','Circle'});

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
            set(obj.timer,'Period',1);
            start(obj.timer);
        end

        function obj = regenerateMap(obj,mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes)
            obj.map = randomMap(mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes);
            obj.mapMsg = rosWriteBinaryOccupancyGrid(obj.mapMsg, obj.map.contents);
            obj.respMap = rosWriteBinaryOccupancyGrid(obj.respMap, obj.map.contents);
        end

        function sendMessage(obj)
            obj.mapMsg.Header.Stamp = rostime('now','DataFormat','struct');
            send(obj.mapPublisher, obj.mapMsg);
        end

        function resp = getMapCallback(obj,~,~,resp)
            obj.respMap.Header.Stamp = rostime('now','DataFormat','struct');
            resp.Map = obj.respMap;
            disp("Map Server: Sending map through service...");
        end

        function delete(obj)
            stop(obj.timer)
            delete(obj.timer)
            delete(obj.mapPublisher)
            delete(obj.server)
            delete(obj.node)
        end
    end
end
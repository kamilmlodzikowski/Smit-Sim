classdef mapServer

    properties
        node
        timer
        publisher
        server
        map
        tftree
    end

    methods
        function obj = mapServer()
            obj.map = randomMap([50 50], 10, 78, 5, 100, {'Box','Plus','Circle'});

            obj.node = ros.Node('matlab/map');

            obj.tftree = ros.TransformationTree(obj.node);

            obj.server = ros.ServiceServer(obj.node,"/static_map","nav_msgs/GetMap",@(~,~,~)obj.getMapCallback,"DataFormat","struct");

            obj.publisher = ros.Publisher(obj.node,'/map', 'nav_msgs/OccupancyGrid','DataFormat','struct');

            obj.timer = timer('Name','MapPublisher');
            set(obj.timer,'executionMode','fixedRate');
            set(obj.timer,'TimerFcn',@(~,~)obj.sendMessage);
            set(obj.timer,'Period',1);
            start(obj.timer);
        end

        function obj = regenerateMap(obj,mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes)
            obj.map = randomMap(mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes);
        end

        function sendMessage(obj)
            %             disp('test');
            message = rosmessage(obj.publisher);
            message.Header.FrameId = 'map';
            message.Header.Stamp = rostime('now','DataFormat','struct');
            message = rosWriteBinaryOccupancyGrid(message, obj.map.contents);
            send(obj.publisher, message);

            tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
            tfStampedMsg.ChildFrameId = 'odom';
            tfStampedMsg.Header.FrameId = 'map';
            tfStampedMsg.Header.Stamp = rostime('now');
            tfStampedMsg.Transform.Rotation.W = 1;
            sendTransform(obj.tftree, tfStampedMsg);
        end

        function resp = getMapCallback(obj,~,~,resp)
            message = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
            message.Header.FrameId = 'map';
            message.Header.Stamp = rostime('now','DataFormat','struct');
            message = rosWriteBinaryOccupancyGrid(message, obj.map.contents);
            resp.Map = message;
            disp("Map Server: Sending map through service...");
        end

        function delete(obj)
            stop(obj.timer)
            delete(obj.timer)
            delete(obj.publisher)
            delete(obj.server)
            delete(obj.node)
        end
    end
end
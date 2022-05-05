classdef mapServer

    properties
        node
        timer
        publisher
        server
        map
    end

    methods
        function obj = mapServer()
            obj.node = ros.Node('matlab/map');
            obj.map = randomMap([50 50], 10, 78, 5, 100, {'Box','Plus','Circle'});

            obj.publisher = rospublisher('/map', 'nav_msgs/OccupancyGrid','DataFormat','struct');

            obj.timer = timer;
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
            message = rosWriteBinaryOccupancyGrid(message, obj.map.contents);
            send(obj.publisher, message);
        end
    end
end
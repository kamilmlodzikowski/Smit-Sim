classdef mapServer < handle

    properties
        node
        server
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
        end

        function walk(obj, sampleTime)
            for i=1:size(obj.pedestrians ,2)
                obj.pedestrians(i).walk(sampleTime);
            end
            obj.updatePedMap()
        end

        function addPedestrian(obj)
            obj.pedestrians = [obj.pedestrians pedestrian(obj.map.contents)];
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

        function resp = getMapCallback(obj,~,~,resp)
            obj.respMap.Header.Stamp = rostime('now','DataFormat','struct');
            resp.Map = obj.respMap;
        end

        function delete(obj)
            delete(obj.server)
            delete(obj.node)
        end
    end
end
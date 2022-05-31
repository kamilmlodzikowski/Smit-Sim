classdef randomMap

    properties
        contents
        mapSize
        resolution
    end

    methods
        function obj = randomMap(mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes)
            % Maze map building
            map1 = mapMaze(passageWidth,wallThickness,'MapSize',mapSize,'MapResolution',resolution);
            grid1 = map1.occupancyMatrix;
            grid2 = map1.occupancyMatrix;
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
            map1 = binaryOccupancyMap(min(grid1,grid2), resolution);

            % Clutter generation
            map2 = mapClutter(numOfClutter,clutterShapes,'MapSize',mapSize,'MapResolution',resolution);
            
            % Map merging
            obj.contents = binaryOccupancyMap(map1.getOccupancy + map2.getOccupancy, resolution);
            obj.mapSize = mapSize;
            obj.resolution = resolution;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end
function map = createRandomMap(mapSize, resolution, passageWidth, wallThickness, numOfClutter, clutterShapes)
%% Parameters
% Map parameters
% mapSize = [50 50];
% resolution = 10;

% Maze generation
% passageWidth = 78; % 94
% wallThickness = 5;

% Clutter generation
% numOfClutter = 100;
% clutterShapes = {'Box','Plus','Circle'};

%% Maze map building
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
% show(map1);

%% Clutter generation
map2 = mapClutter(numOfClutter,clutterShapes,'MapSize',mapSize,'MapResolution',resolution);
% show(map2);

%% Map merging
map = binaryOccupancyMap(map1.getOccupancy + map2.getOccupancy, resolution);
% show(map);
end
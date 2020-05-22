clear all; close all;

MyMap = double(imread('MyMap.png'));
MyMap = MyMap(:,:,1)<2;
whos *Map*
map = binaryOccupancyMap(MyMap,10);
inflate(map, 0.1)
figure(1);  show(map);  grid on;

prm = mobileRobotPRM;   % PRM = Prob. Road Map
prm.Map = map;
prm.NumNodes = 100;
prm.ConnectionDistance = 5;

% Find a Feasible Path on the Constructed PRM
startLocation = [1 0];
endLocation = [8 8];
path = findpath(prm, startLocation, endLocation)

figure(3);  show(prm); grid on

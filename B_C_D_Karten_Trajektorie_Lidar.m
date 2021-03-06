%% Kartenerstellung_Trajektorie_Lidarscan
%  =========================
%  A. Same     27.04.2020
%  =========================

clc; clear all; close all;
disp('Kartenerstellung, Trajektorie und Lidar Sensor');

%% Create and Modify Binary Occupancy Grid
% Set occupancy of world locations and show map.
MyMap = double(imread('MyMap.png'));
MyMap = MyMap(:,:,1)<2;
whos *Map*
map = binaryOccupancyMap(MyMap,10);
figure(1);  show(map);  grid on;
            % Inflate occupied locations by a given radius. 
robotRadius = .2;
inflate(map, robotRadius)
figure(2); show(map); grid on
            title('die erreichbaren Bereiche');
            
%% Trajektorie mit Start-, Endpunkt und 8 Wegpunkten
% Time, Waypoint, Orientation
n = 20;
%  Info       t,   x,   y, z, Eul. x,          y, z [deg]
trajInfo = [  0,   1,   0, 0,      pi/2,        0, 0; ... % Start point and Initial position
              1,   1,   1, 0,      deg2rad(80), 0, 0; ...
              2,  1.5,  2, 0,      deg2rad(80), 0, 0; ...
              3,   2,   3, 0,      deg2rad(90), 0, 0; ...
              4,   2,   4, 0,      deg2rad(60), 0, 0; ...
              5,  3.5,  5, 0,      deg2rad(40), 0, 0; ...
              6,  4.5, 5.5,0,      deg2rad(0),  0, 0; ...
              7,   5,   5, 0,      deg2rad(-30),0, 0; ...
              8,  6.5,  5, 0,      deg2rad(80), 0, 0; ...
              9,  6.7,  6, 0,      deg2rad(85), 0, 0; ...
              10,  8,   8, 0,      pi/2,        0, 0];    % Stop point and Final position

trajectory = waypointTrajectory(trajInfo(:,2:4), ...
    'TimeOfArrival',trajInfo(:,1), ...
    'Orientation',quaternion(trajInfo(:,5:end),'eulerd','ZYX','frame'), ...    
    'SampleRate',n);

count = 1;
while ~isDone(trajectory)
   [position(count,:), orientation(count,:), velocity(count,:), acceleration(count,:), angularVelocity(count,:)] = trajectory();
   count = count+1;
end
   count = count-1;
   
% Position [m] und Winkel [deg]
eulerAngles = eulerd([orientation],'ZYX','frame');


% addint noise to position vectors
position(:,1)= position(:,1)+0.1*rand(length(position(:,1)),1);
position(:,2)= position(:,2)+0.1*rand(length(position(:,2)),1);
% Plot
hold on;    plot(position(:,1),position(:,2),'bo');
            title('Position'); xlabel('X'); ylabel('Y'); 
            axis square; grid on;
%             hold on;
            plot(trajInfo(:,2),trajInfo(:,3),'rx');
            legend('Trajectory', 'Defined points'); hold off;
figure(3);  plot(eulerAngles(:,1),'b.');
            xlabel('Time [1/10 s]'); ylabel('Angular velocity [rad/s]');
            axis square; grid on;
            
figure(4);  plot(velocity(:,1),'k-'); grid on; hold on
            plot(velocity(:,2),'r-'); hold off
            legend('X velocity','Y Velocity');
            xlabel('Time [1/10 s]');
%% Lidar-Sensor
rbsensor = rangeSensor; % Sensor-Definition
x = position(:,1);
y = position(:,2);

figure(5);  plot(x,y,'rx'); axis equal;
rbsensor.HorizontalAngleResolution = deg2rad(1);
rbsensor.Range=5;
release(rbsensor);
rbsensor.HorizontalAngle=[-pi/2 pi/2];
figure(6);  hold on;
for k = 1:length(x)
    truePose = [x(k) y(k) eulerAngles(k)];
                        % Sensor-Pose X,Y,Winkel
    [ranges, angles] = rbsensor(truePose, map);
                        % Sensor-Werte
    MyLidarScan(k) = lidarScan(ranges, angles);
                        % Zuweisung Lidar-Objekt
    plot(MyLidarScan(k))       % Darstellung 
end

title('Lidar Multi-Scan'); axis equal;

% Saving the lidar scans for further use
save('B_C_D_Karten_Trajektorie_Lidar.mat','MyLidarScan')
save('eulerAngles','eulerAngles')

%% Mobile Robotik Simulation 
%  =========================
%  Map 03.) Roboter-Pose Scan-Matching
%  A. Same     22.05.2020
%  =========================
% https://de.mathworks.com/help/nav/ug/estimate-robot-pose-with-scan-matching.html


%% %%%%%IMPORTANT%%%%%IMPORTANT%%%%%%IMPORTANT%%%%%IMPORTANT%%%%%%IMPORTANT%%%%% IMPORTANT %%
%                                                                                          %%
%  NOTE: please run "B_C_D_Karten_Trajektorie_Lidar.m" file before executing this program. %%
%                                                                                          %%
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
clc; clear all; close all;
disp('Map 03: Roboter-Pose Scan-Matching');

%% Loading lidar sensor data,collected from ""B_C_D_Karten_Trajektorie_Lidar.m" file
load('B_C_D_Karten_Trajektorie_Lidar.mat')
load('eulerAngles')
lidarScans = MyLidaScan;

%% Estimate Robot Pose with Scan Matching
referenceScan = lidarScans(1);
currentScan = lidarScans(2);

refScanCart = referenceScan.Cartesian; % Reading the saved data, which are collected with sensor
currScanCart = currentScan.Cartesian;

figure(1);
    plot(refScanCart(:,1),refScanCart(:,2),'k.'); hold on
    plot(currScanCart(:,1),currScanCart(:,2),'r.');
    legend('Reference laser scan','Current laser scan','Location','NorthWest');

%% Run Scan Matching Algorithm and Display Transformed Scan
transform = matchScans(currentScan,referenceScan);
transScan =  transformScan(currentScan,transform);

figure(2);
    plot(refScanCart(:,1),refScanCart(:,2),'k.');   hold on
    transScanCart = transScan.Cartesian;
    plot(transScanCart(:,1),transScanCart(:,2),'r.');
    legend('Reference laser scan','Transformed current laser scan','Location','NorthWest');

% Build Occupancy Grid Map Using Iterative Scan Matching
map = occupancyMap(20,20,20);
map.GridLocationInWorld = [-5 -10];

% Pre-allocate an array to capture the absolute movement of the robot.
% Initialize the first pose as [0 0 0]. All other poses are relative to the first measured scan. 
numScans = numel(lidarScans);
initialPose = [pi/2 0 0];
poseList = eulerAngles;%zeros(numScans,3);
poseList(1,:) = initialPose;
transform = initialPose;

% Create a loop for processing the scans and mapping the area. The laser scans are processed in pairs.
% Define the first scan as reference scan and the second scan as current scan. The two scans are then
% passed to the scan matching algorithm and the relative pose between the two scans is computed.
% The exampleHelperComposeTransform function is used to calculate of the cumulative absolute robot pose.
% The scan data along with the absolute robot pose can then be passed into the insertRay function of the occupancy grid.
% Loop through all the scans and calculate the relative poses between them
for idx = 2:numScans
    % Process the data in pairs.
    referenceScan = lidarScans(idx-1);
    currentScan = lidarScans(idx);
    %
    % Run scan matching. Note that the scan angles stay the same and do 
    % not have to be recomputed. To increase accuracy, set the maximum 
    % number of iterations to 500. Use the transform from the last
    % iteration as the initial estimate.
    [transform,stats] = matchScans(currentScan,referenceScan, ...
        'MaxIterations',500); % no need for initial pose
     
    % The |Score| in the statistics structure is a good indication of the
    % quality of the scan match. 
    if stats.Score / currentScan.Count < 1.0
        disp(['Low scan match score for index ' num2str(idx) '. Score = ' num2str(stats.Score) '.']);
    end
    
    % Maintain the list of robot poses. 
    absolutePose = exampleHelperComposeTransform(poseList(idx-1,:),transform);
    poseList(idx,:) = absolutePose;
       
    % Integrate the current laser scan into the probabilistic occupancy
    % grid.
    insertRay(map,absolutePose,currentScan,10);

end

% Visualize Map
figure(3);  show(map);
    title('Occupancy grid map built using scan matching results'); hold on
    plot(poseList(:,1),poseList(:,2),'bo','DisplayName','Estimated robot position');
    legend('show','Location','NorthWest')
  

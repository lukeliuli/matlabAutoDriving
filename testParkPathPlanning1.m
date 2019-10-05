clc;
close all;
clear all;
%https://ww2.mathworks.cn/help/driving/examples/automated-parking-valet.html
mapLayers = loadParkingLotMapLayers;
plotMapLayers(mapLayers)

costmap = combineMapLayers(mapLayers);

figure
plot(costmap, 'Inflation', 'off')
legend off


costmap.MapExtent % [x, width, y, height] in meters

costmap.CellSize  % cell size in meters

vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees
costmap.CollisionChecker.VehicleDimensions = vehicleDims;
currentPose = [4 12 0]; % [x, y, theta]
data = load('routePlan.mat');
routePlan = data.routePlan 

hold on
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend




for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};

    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

behavioralPlanner = HelperBehavioralPlanner(routePlan, maxSteeringAngle);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance', 10, 'MinTurningRadius', 20);
goalPose = routePlan{1, 'EndPose'};
refPath = plan(motionPlanner, currentPose, goalPose);
refPath.PathSegments
[transitionPoses, directions] = interpolate(refPath);

% Visualize the planned path
plot(motionPlanner)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Specify number of poses to return using a separation of approximately 0.1 m
approxSeparation = 0.1; % meters
numSmoothPoses   = round(refPath.Length / approxSeparation);

% Return discretized poses along the smooth path
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);

% Plot the smoothed path
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
maxSpeed   = 5; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second

%Generate a velocity profile
refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
plotVelocityProfile(cumLengths, refVelocities, maxSpeed)


% Close all the figures
closeFigures;

% Create the vehicle simulator
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

% Set the vehicle pose and velocity 
vehicleSim.setVehiclePose(currentPose);
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

% Configure the simulator to show the trajectory
vehicleSim.showTrajectory(true);

% Hide vehicle simulation figure
hideFigure(vehicleSim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pathAnalyzer = HelperPathAnalyzer(refPoses, refVelocities, directions, ...
    'Wheelbase', vehicleDims.Wheelbase);

sampleTime = 0.05;
lonController = HelperLongitudinalController('SampleTime', sampleTime);

controlRate = HelperFixedRate(1/sampleTime); % in Hertz

reachGoal = false;

while ~reachGoal    
    % Find the reference pose on the path and the corresponding velocity
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    % Update driving direction for the simulator
    updateDrivingDirection(vehicleSim, direction);
    
    % Compute steering command
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    
    % Compute acceleration and deceleration commands
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simulate the vehicle using the controller outputs
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Check if the vehicle reaches the goal
    reachGoal = helperGoalChecker(goalPose, currentPose, currentVel, endSpeed, direction);
    
    % Wait for fixed-rate execution
    waitfor(controlRate);
    
    % Get current pose and velocity of the vehicle
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end

% Show vehicle simulation figure
showFigure(vehicleSim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the vehicle pose back to the initial starting point
currentPose = [4 12 0]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Reset velocity
currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);

while ~reachedDestination(behavioralPlanner)
    
    % Request next maneuver from behavioral layer
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
        currentPose, currentVel);
    
    % Configure the motion planner
    configurePlanner(motionPlanner, plannerConfig);
    
    % Plan a reference path using RRT* planner to the next goal pose
    refPath = plan(motionPlanner, currentPose, nextGoal);
    
    % Check if the path is valid. If the planner fails to compute a path,
    % or the path is not collision-free because of updates to the map, the
    % system needs to re-plan. This scenario uses a static map, so the path
    % will always be collision-free.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Request behavioral planner to re-plan
        replanNeeded(behavioralPlanner);
        continue;
    end
    
    % Retrieve transition poses and directions from the planned path
    [transitionPoses, directions] = interpolate(refPath);
     
    % Smooth the path
    numSmoothPoses   = round(refPath.Length / approxSeparation);
    [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
    
    % Generate a velocity profile
    refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
    
    % Configure path analyzer
    pathAnalyzer.RefPoses     = refPoses;
    pathAnalyzer.Directions   = directions;
    pathAnalyzer.VelocityProfile = refVelocities;
    
    % Reset longitudinal controller 
    reset(lonController);
    
    reachGoal = false;
    
    % Execute control loop
    while ~reachGoal  
        % Find the reference pose on the path and the corresponding velocity
        [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
        
        % Update driving direction for the simulator
        updateDrivingDirection(vehicleSim, direction);
        
        % Compute steering command
        steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
            'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
        
        % Compute acceleration and deceleration commands
        lonController.Direction = direction;
        [accelCmd, decelCmd] = lonController(refVel, currentVel);
        
        % Simulate the vehicle using the controller outputs
        drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
        
        % Check if the vehicle reaches the goal
        reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
        
        % Wait for fixed-rate execution
        waitfor(controlRate);
        
        % Get current pose and velocity of the vehicle
        currentPose  = getVehiclePose(vehicleSim);
        currentVel   = getVehicleVelocity(vehicleSim);
    end
end

% Show vehicle simulation figure
showFigure(vehicleSim);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function mapLayers = loadParkingLotMapLayers()
%loadParkingLotMapLayers
%   Load occupancy maps corresponding to 3 layers - obstacles, road
%   markings, and used spots.

mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end


function plotMapLayers(mapLayers)
%plotMapLayers
%   Plot the multiple map layers on a figure window.

figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end

function costmap = combineMapLayers(mapLayers)
%combineMapLayers
%   Combine map layers struct into a single vehicleCostmap.

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end

function plotVelocityProfile(cumPathLength, refVelocities, maxSpeed)
%plotVelocityProfile
% Plot the generated velocity profile

% Plot reference velocity along length of the path
plot(cumPathLength, refVelocities, 'LineWidth', 2);

% Plot a line to display maximum speed
hold on
line([0;cumPathLength(end)], [maxSpeed;maxSpeed], 'Color', 'r')
hold off

% Set axes limits
buffer = 2;
xlim([0 cumPathLength(end)]);
ylim([0 maxSpeed + buffer])

% Add labels
xlabel('Cumulative Path Length (m)');
ylabel('Velocity (m/s)');

% Add legend and title
legend('Velocity Profile', 'Max Speed')
title('Generated velocity profile')
end


function configurePlanner(pathPlanner, config)
%configurePlanner
% Configure the path planner object, pathPlanner, with settings specified
% in struct config.

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    if ~strcmpi(fieldNames{n}, 'IsParkManeuver')
        pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
    end
end
end

function closeFigures()
% Close all the figures except the simulator visualization

% Find all the figure objects
figHandles = findobj('Type', 'figure');
for i = 1: length(figHandles)
    if ~strcmp(figHandles(i).Name, 'Automated Valet Parking')
        close(figHandles(i));
    end
end
end
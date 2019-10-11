%ʵ���Զ����ʻ����,�ο�https://www.mathworks.com/help/releases/R2019b/driving/examples/driving-scenario-tutorial.html
function mainTestDS1

scenario = drivingScenario;
v1 = vehicle(scenario,'ClassID',1','Position',[6 0 0],'Velocity',[-3 0 0],'Yaw',180);
v2 = vehicle(scenario,'ClassID',1,'Position',[0 10 0],'Velocity',[0 -4 0],'Yaw',-90);

plot(scenario);
set(gcf,'Name','Scenario Plot')
xlim([-20 20]);
ylim([-20 20]);

ap = actorPoses(scenario)
v2TargetPoses = targetPoses(v2)

chasePlot(v2)
set(gcf,'Name','Chase Plot')

v1.Yaw = 135;
updatePlots(scenario);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scenario = drivingScenario;
roadCenters = ...
    [  0  40  49  50 100  50  49 40 -40 -49 -50 -100  -50  -49  -40    0
     -50 -50 -50 -50   0  50  50 50  50  50  50    0  -50  -50  -50  -50
       0   0 .45 .45 .45 .45 .45  0   0 .45 .45  .45  .45  .45    0    0]';
bankAngles = ...
    [  0   0   9   9   9   9   9  0   0   9   9    9    9    9    0    0];

road(scenario, roadCenters, bankAngles, 'lanes', lanespec(2));
plot(scenario)

rb = roadBoundaries(scenario)


figure

outerBoundary = rb{1};
innerBoundary = rb{2};

plot3(innerBoundary(:,1),innerBoundary(:,2),innerBoundary(:,3),'r', ...
      outerBoundary(:,1),outerBoundary(:,2),outerBoundary(:,3),'g')
axis equal

egoCar = vehicle(scenario,'ClassID',1,'Position',[80 -40 0.45],'Yaw',30);

figure

rb = roadBoundaries(egoCar)
outerBoundary = rb{1};
innerBoundary = rb{2};

plot3(innerBoundary(:,1),innerBoundary(:,2),innerBoundary(:,3),'r', ...
      outerBoundary(:,1),outerBoundary(:,2),outerBoundary(:,3),'g')
axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
chasePlot(egoCar);
fastCar = vehicle(scenario,'ClassID',1);

d = 2.7/2;
h = .45/2;
roadOffset = [ 0  0  0  0  d  0  0  0  0  0  0 -d  0  0  0  0
              -d -d -d -d  0  d  d  d  d  d  d  0 -d -d -d -d
               0  0  h  h  h  h  h  0  0  h  h  h  h  h  0  0]';

rWayPoints = roadCenters + roadOffset;
lWayPoints = roadCenters - roadOffset;

% loop around the track four times
rWayPoints = [repmat(rWayPoints(1:end-1,:),5,1); rWayPoints(1,:)];
lWayPoints = [repmat(lWayPoints(1:end-1,:),5,1); lWayPoints(1,:)];

trajectory(egoCar,rWayPoints(:,:), 30);
trajectory(fastCar,lWayPoints(:,:), 50);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scenario.SampleTime = 0.02;
scenario.StopTime = 4;
while advance(scenario)
  pause(0.001)
end

close all

scenario.StopTime = 0.100;
poseRecord = record(scenario)

r = poseRecord(5)
r.ActorPoses(1)
r.ActorPoses(2)

close all;
hFigure = figure;
hFigure.Position(3) = 900;

hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 1/4 1/2 3/4],'Title','Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/2 1/4],'Title','Chase Plot');
hPanel3 = uipanel(hFigure,'Units','Normalized','Position',[1/2 0 1/2 1],'Title','Bird''s-Eye Plot');

hAxes1 = axes('Parent',hPanel1);
hAxes2 = axes('Parent',hPanel2);
hAxes3 = axes('Parent',hPanel3);

% assign a scenario plot and a chase plot in the first two axes
plot(scenario, 'Parent', hAxes1);
chasePlot(egoCar, 'Parent', hAxes2);

% assign a bird's-eye plot in third axes.
egoCarBEP = birdsEyePlot('Parent',hAxes3,'XLimits',[-200 200],'YLimits',[-240 240]);
fastTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','red','DisplayName','target','VelocityScaling',.5);
egoTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','blue','DisplayName','ego','VelocityScaling',.5);
egoLanePlotter = laneBoundaryPlotter(egoCarBEP);
plotTrack(egoTrackPlotter, [0 0]);
egoOutlinePlotter = outlinePlotter(egoCarBEP);

restart(scenario)
scenario.StopTime = Inf;

while advance(scenario)
    t = targetPoses(egoCar);
    plotTrack(fastTrackPlotter, t.Position, t.Velocity);
    rbs = roadBoundaries(egoCar);
    plotLaneBoundary(egoLanePlotter, rbs);
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(egoOutlinePlotter, position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
end
end
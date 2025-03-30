clc; clear all; close all;

%% Inicjalizacja scenariusza
scenario = robotScenario(UpdateRate=100);

% Konfiguracja środowiska
floorColor = [0.5882 0.2941 0];
addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=floorColor);

% Ściany
wallHeight = 1; wallWidth = 0.25; wallLength = 10; wallColor = [1 1 0.8157];
addMesh(scenario, "Box", Position=[wallWidth/2, wallLength/2, wallHeight/2], ...
    Size=[wallWidth, wallLength, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength-wallWidth/2, wallLength/2, wallHeight/2], ...
    Size=[wallWidth, wallLength, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/2, wallLength-wallWidth/2, wallHeight/2], ...
    Size=[wallLength, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/2, wallWidth/2, wallHeight/2], ...
    Size=[wallLength, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);

% Wyświetlenie sceny
show3D(scenario); lightangle(-45,30); view(60,60);

%% Planowanie trasy
map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[15 15], MapHeightLimits=[0 3]);
inflate(map,0.3);
startPosition = [1 1]; goalPosition = [8 8];
planner = mobileRobotPRM(map, 3); planner.ConnectionDistance = 5;
waypoints = findpath(planner, startPosition, goalPosition);

%% Konfiguracja robota i czujników
robotheight = 0.12;
traj = waypointTrajectory(SampleRate=100, TimeOfArrival=0:(size(waypoints,1)-1), ...
    Waypoints=[waypoints, robotheight*ones(size(waypoints,1),1)], ReferenceFrame="ENU");

huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=traj);

lidarModel = robotLidarPointCloudGenerator(UpdateRate=100, MaxRange=10, RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ElevationResolution=1.25, AzimuthLimits=[-180 180], ElevationLimits=[0 10]);
lidar = robotSensor("lidar", platform, lidarModel, MountingLocation=[0 0 0.3]);

%% Główna pętla symulacji - zbieranie danych
setup(scenario);
simData = struct('scans', {}, 'poses', [], 'timestamps', []);
r = rateControl(20);

while advance(scenario)
    [~, ~, currentPC] = read(lidar);
    currentPose = read(platform);
    updateSensors(scenario);
    
    if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
        simData(end+1).scans = currentPC;
        simData(end).poses = currentPose;
        simData(end).timestamps = scenario.CurrentTime;
    end
    
    show3D(scenario, FastUpdate=true);
    waitfor(r);
    
    if scenario.CurrentTime > 20  % Warunek zakończenia
        break;
    end
end

%% Zapis danych do pliku MAT
save('simulation_data.mat', 'simData', 'waypoints', 'map');
disp('Dane symulacji zapisane w simulation_data.mat');
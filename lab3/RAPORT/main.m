clc;clear all;
%% Inicjalizacja scenariusza
scenario = robotScenario(UpdateRate=100);

% Kolor podłoża
floorColor = [0.5882 0.2941 0];
addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=floorColor);

% Parametry ścian
wallHeight = 1;
wallWidth = 0.25;
wallLength = 10;
wallColor = [1 1 0.8157];

% Dodanie zewnętrznych ścian
addMesh(scenario, "Box", Position=[wallWidth/2, wallLength/2, wallHeight/2], ...
    Size=[wallWidth, wallLength, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength-wallWidth/2, wallLength/2, wallHeight/2], ...
    Size=[wallWidth, wallLength, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/2, wallLength-wallWidth/2, wallHeight/2], ...
    Size=[wallLength, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/2, wallWidth/2, wallHeight/2], ...
    Size=[wallLength, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);

% % Dodanie wewnętrznych ścian
addMesh(scenario, "Box", Position=[wallLength/8, wallLength/3, wallHeight/2], ...
    Size=[wallLength/4, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/4, wallLength/3, wallHeight/2], ...
    Size=[wallWidth, wallLength/6, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[(wallLength-wallLength/4), wallLength/2, wallHeight/2], ...
    Size=[wallLength/2, wallWidth, wallHeight], Color=wallColor, IsBinaryOccupied=true);
addMesh(scenario, "Box", Position=[wallLength/2, wallLength/2, wallHeight/2], ...
    Size=[wallWidth, wallLength/3, wallHeight], Color=wallColor, IsBinaryOccupied=true);


% Wyświetlenie sceny
show3D(scenario);
lightangle(-45,30);
view(60,60);

%% Tworzenie mapy binarnej
map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], ...
    MapSize=[15 15], MapHeightLimits=[0 3]);
inflate(map,0.5);
show(map);

%% Planowanie trasy
startPosition = [1.5 1.5];
goalPosition = [8 8];

numnodes = 300;
planner = mobileRobotPRM(map, numnodes);
planner.ConnectionDistance = 10;
waypoints = findpath(planner, startPosition, goalPosition);

%% Robot
robotheight = 0.12;
numWaypoints = size(waypoints,1);
firstInTime = 0;
lastInTime = firstInTime + (numWaypoints-1);

traj = waypointTrajectory(SampleRate=100, ...
    TimeOfArrival=firstInTime:lastInTime, ...
    Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ...
    ReferenceFrame="ENU");

huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, ...
    BaseTrajectory=traj);


%% Sensor LIDAR - zaktualizowana konfiguracja 3D\
Lidar_Range = 5;

lidarModel = robotLidarPointCloudGenerator(...
    UpdateRate=100, ...
    MaxRange=5, ...
    RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ...
    ElevationResolution=1.25, ...
    AzimuthLimits=[-180 180], ...
    ElevationLimits=[0 10], ...  
    HasNoise=false, ...
    HasOrganizedOutput=true);

lidar = robotSensor("lidar", platform, lidarModel, ...
    MountingLocation=[0 0 0.3], MountingAngles=[0 0 0],UpdateRate=100);

%% Inicjalizacja SLAM
maxLidarRange = 10;       
mapResolution = 20;      

% Inicjalizacja obiektu SLAM
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
slamAlg.OptimizationInterval = 10;

% Inicjalizacja mapy zajętości
occupancyMap = occupancyMap(20, 20, 20);

%% Uruchomienie symulacji
setup(scenario);

figure('Name', 'Robot Simulation', 'NumberTitle', 'off');
[ax, plotFrames] = show3D(scenario);
lightangle(-45,30);
view(60,60);
hold(ax, "on");
plot(ax, waypoints(:,1), waypoints(:,2), "-ms", ...
    LineWidth=2, ...
    MarkerSize=4, ...
    MarkerEdgeColor="b", ...
    MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax, "off");

r = rateControl(20);
robotStartMoving = false;

% Tablice do przechowywania danych
robotPoses = [];
globalPointCloud = pointCloud(zeros(0,3));
timestamps = [];           
disp("Rozpoczynam symulację...");

initialPosition = [];

%% Pętla główna
scanCount = 0;
while advance(scenario)
    % Odczyt danych z LIDAR-a i INS
    [~, ~, currentPC] = read(lidar);
    currentPose = read(platform);
    updateSensors(scenario);
    advance(scenario);
    
    if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
        % Ekstrakcja danych
        position = currentPose(1:3);
        quaternion = currentPose(10:13);
        
        % Inicjalizacja początkowej pozycji
        if isempty(initialPosition)
            initialPosition = position;
        end
        
        % Przetwarzanie chmury punktów
        xyzPoints = reshape(currentPC.Location, [], 3);
        validPoints = ~any(isnan(xyzPoints), 2);
        xyzPoints = xyzPoints(validPoints, :);
        xyzPoints = xyzPoints + [0 0 0.3]; % LIDAR offset
        
        scan = lidarScan(xyzPoints(:,1:2));
        
        % Dodanie skanu do SLAM
            if ~isempty(scan.Ranges)
                % skan co 5 iteracji
                if mod(scanCount, 15) == 0
                    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
                    
                    % Aktualizacja wizualizacji SLAM
                    if isScanAccepted
                    end
                end
                
                scanCount = scanCount + 1;
            end
        
        % Transformacja chmury punktów do układu globalnego
        try
            R = quat2rotm(quaternion);
            rotTform = rotm2tform(R);
            transTform = trvec2tform(position);
            tform = transTform * rotTform;
            
            transformedPoints = zeros(size(xyzPoints));
            for i = 1:size(xyzPoints, 1)
                homogeneousPoint = [xyzPoints(i,:), 1]';
                transformedHomogeneous = tform * homogeneousPoint;
                transformedPoints(i,:) = transformedHomogeneous(1:3)';
            end
            
            globalPointCloud = pointCloud([globalPointCloud.Location; transformedPoints]);
            disp("ok");
        catch ME
            disp('Błąd transformacji:');
            disp(ME.message);
        end
    end
    
    % Aktualizacja wizualizacji 3D
    show3D(scenario, Parent=ax, FastUpdate=true);
    waitfor(r);
    
    if ~any(isnan(currentPose))
        robotStartMoving = true;
    end
    
    if any(isnan(currentPose)) && robotStartMoving
        break;
    end
end

disp("Symulacja zakończona. Przetwarzam dane...");

%% Finalizacja SLAM
% Wykonanie ostatniej optymalizacji

poseGraph = slamAlg.PoseGraph;
optimizePoseGraph(poseGraph);

% Pobranie ostatecznej mapy zajętości
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

%% Wizualizacja wyników
figure;
pcshow(globalPointCloud);
title('Globalna chmura punktów po transformacji');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

% Utworzenie wspólnej figury dla mapy zajętości i grafu pozycji
figure;
ax1 = axes;
show(map, 'Parent', ax1);
title('Mapa zajętości z SLAM i graf pozycji');
xlabel('X [m]');
ylabel('Y [m]');
hold on;

% Nałożenie grafu pozycji na mapę zajętości
show(slamAlg.PoseGraph, 'Parent', ax1);
hold off;
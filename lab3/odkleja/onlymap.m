clear all;
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

% Dodanie wewnętrznych ścian
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
inflate(map,0.3);
show(map);

%% Planowanie trasy
startPosition = [1 1];
goalPosition = [8 8];

numnodes = 500;
planner = mobileRobotPRM(map, numnodes);
planner.ConnectionDistance = 1;
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

%% SENSOR INS
insModel = insSensor(YawAccuracy=10,PitchAccuracy=10,RollAccuracy=10,HasGNSSFix=true);

%% Sensor LIDAR - zaktualizowana konfiguracja 3D
lidarModel = robotLidarPointCloudGenerator(...
    UpdateRate=100, ...
    MaxRange=5, ...
    RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ...
    ElevationResolution=1.25, ...
    AzimuthLimits=[-180 180], ... % 180° w poziomie
    ElevationLimits=[0 10], ...  % Skanowanie w zakresie -15° do +15° w pionie
    HasNoise=false, ...
    HasOrganizedOutput=true);

lidar = robotSensor("lidar", platform, lidarModel, ...
    MountingLocation=[0 0 0.3], MountingAngles=[0 0 0], UpdateRate=100);
%% Uruchomienie symulacji

% Ustawienie scenariusza
setup(scenario);

% Tworzenie wizualizacji PRZED pętlą główną
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
globalPointCloud = pointCloud(zeros(0,3)); % Inicjalizacja pustej chmury punktów
timestamps = [];           % Przechowuje czasy dla każdego skanu
disp("Rozpoczynam symulację...");

%ins: pozyje yxz

% Ustal początkową pozycję robota (na początku symulacji)
initialPosition = [];


%% Pętla główna
while advance(scenario)
    % Odczyt danych z LIDAR-a i INS
    [~, ~, currentPC] = read(lidar);
    currentPose = read(platform);
    updateSensors(scenario);
    advance(scenario);
    
    % Debugging and robustness modifications
    if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
        % Extract position and quaternion from currentPose
        position = currentPose(1:3);         % [x y z]
        quaternion = currentPose(10:13);     % [qw qx qy qz]
        
        % Optional: Display position and quaternion for debugging
        fprintf('Position (x,y,z): [%.3f, %.3f, %.3f]\n', position(1), position(2), position(3));
        fprintf('Quaternion (w,x,y,z): [%.3f, %.3f, %.3f, %.3f]\n', quaternion(1), quaternion(2), quaternion(3), quaternion(4));
        
        % Initialize initial position if needed
        if isempty(initialPosition)
            initialPosition = position;
        end
        
        % 1. Get raw LIDAR point cloud (skip NaN)
        xyzPoints = reshape(currentPC.Location, [], 3);
        validPoints = ~any(isnan(xyzPoints), 2);
        xyzPoints = xyzPoints(validPoints, :);
        
        % 2. Account for LIDAR offset
        lidarOffset = [0 0 0.3];
        xyzPoints = xyzPoints + lidarOffset;
        
        try
    % Konwersja quaternionu na macierz rotacji
    R = quat2rotm(quaternion);
    
    % Utworzenie macierzy transformacji dla rotacji i translacji
    rotTform = rotm2tform(R);
    transTform = trvec2tform(position);
    
    % Połączenie transformacji (rotacja, następnie translacja)
    tform = transTform * rotTform;
    
    % Transformacja punktów
    transformedPoints = zeros(size(xyzPoints));
    for i = 1:size(xyzPoints, 1)
        homogeneousPoint = [xyzPoints(i,:), 1]';
        transformedHomogeneous = tform * homogeneousPoint;
        transformedPoints(i,:) = transformedHomogeneous(1:3)';
    end
    
    globalPointCloud = pointCloud([globalPointCloud.Location; transformedPoints]);
    robotPoses = [robotPoses; position];
catch ME
    disp('Error in transformation:');
    disp(ME.message);
end
    end
    
    % Update 3D visualization
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

%% Wizualizacja wynikowej chmury punktów
figure;
pcshow(globalPointCloud);
title('Globalna chmura punktów po transformacji');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

%% Slam



%% SLAM

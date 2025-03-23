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

%% Definicja własnych punktów trasy
% Punkty trasy zgodnie z żądaniem: 1,1 -> 9,1 -> 9,4 -> 5,4 -> 4,0 -> 4,5 -> 1,5 -> 1,9 -> 9,9 -> 6,9 -> 6,6
baseWaypoints = [
    1, 1;
    9, 1;
    9, 4;
    6, 4;
    4, 2;
    4, 5;
    1, 5;
    1, 9;
    9, 9;
    9, 6;
    6, 6
];

% Generowanie dodatkowych punktów pośrednich dla ruchu po liniach prostych
customWaypoints = [];
numPointsPerSegment = 2;  % Liczba punktów pośrednich na segment

for i = 1:size(baseWaypoints, 1)-1
    p1 = baseWaypoints(i, :);
    p2 = baseWaypoints(i+1, :);
    
    % Generuj punkty pośrednie na prostej między p1 i p2
    for j = 0:numPointsPerSegment
        t = j / numPointsPerSegment;
        intermediatePoint = p1 * (1-t) + p2 * t;
        customWaypoints = [customWaypoints; intermediatePoint];
    end
end

% Dodanie ostatniego punktu
customWaypoints = [customWaypoints; baseWaypoints(end, :)];

% Sprawdzenie czy punkty są poza przeszkodami
for i = 1:size(customWaypoints, 1)
    x = customWaypoints(i, 1);
    y = customWaypoints(i, 2);
    if getOccupancy(map, [x, y]) > 0.5
        warning('Punkt [%d, %d] koliduje z przeszkodą!', x, y);
    end
end

%% Robot
robotheight = 0.12;
numWaypoints = size(customWaypoints, 1);
firstInTime = 0;
lastInTime = firstInTime + (numWaypoints-1);

% Definiujemy czasy przybycia dla każdego punktu - równomierne rozłożenie w czasie
timeOfArrival = linspace(firstInTime, lastInTime, numWaypoints);

% Używamy waypointTrajectory bez opcji Orientation
traj = waypointTrajectory(SampleRate=20, ...
    TimeOfArrival=timeOfArrival, ...
    Waypoints=[customWaypoints, robotheight*ones(numWaypoints,1)], ...
    ReferenceFrame="ENU");

huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, ...
    BaseTrajectory=traj);

%% Sensor LIDAR - zaktualizowana konfiguracja 3D
lidarModel = robotLidarPointCloudGenerator(...
    UpdateRate=100, ...
    MaxRange=12, ...
    RangeAccuracy=0.01, ...
    AzimuthResolution=0.05, ...
    ElevationResolution=0.05, ...
    AzimuthLimits=[-90 90], ... % 180° w poziomie
    ElevationLimits=[0 15], ...  % Skanowanie w zakresie -15° do +15° w pionie
    HasNoise=false, ...
    HasOrganizedOutput=false);

lidar = robotSensor("lidar", platform, lidarModel, ...
    MountingLocation=[0 0 0.5]);

%% Uruchomienie symulacji

% Ustawienie scenariusza
setup(scenario);

% Tworzenie wizualizacji PRZED pętlą główną
figure('Name', 'Robot Simulation', 'NumberTitle', 'off');
[ax, plotFrames] = show3D(scenario);
lightangle(-45,30);
view(60,60);
hold(ax, "on");

% Narysowanie oryginalnych punktów bazowych
plot(ax, baseWaypoints(:,1), baseWaypoints(:,2), "-ms", ...
    LineWidth=2, ...
    MarkerSize=8, ...
    MarkerEdgeColor="b", ...
    MarkerFaceColor=[0.5 0.5 0.5]);

% Narysowanie linii prostych między punktami (bez punktów pośrednich)
plot(ax, customWaypoints(:,1), customWaypoints(:,2), "--r", ...
    LineWidth=1);

hold(ax, "off");

r = rateControl(20);
robotStartMoving = false;

% Tablice do przechowywania danych
robotPoses = [];
scanBuffer = cell(0);      % Przechowuje chmury punktów 3D
lidarScans = cell(0);      % Przechowuje skany 2D dla SLAM
timestamps = [];           % Przechowuje czasy dla każdego skanu

disp("Rozpoczynam symulację...");

while advance(scenario)
    
    % Odczyt danych z LIDAR-a
    [~, ~, currentPC] = read(lidar);
    
    % Odczyt aktualnej pozycji robota
    currentPose = read(platform);
    
    if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
        disp("Valid LIDAR data found at time: " + scenario.CurrentTime);
        
        % Zapisz chmurę punktów 3D
        scanBuffer{end+1} = currentPC;
        
        % Projekcja chmury punktów 3D na 2D dla SLAM
        pcLocs = currentPC.Location;
        xyPoints = pcLocs(:, 1:2);
        
        % Konwersja na obiekt lidarScan wymagany przez algorytm SLAM
        ranges = sqrt(sum(xyPoints.^2, 2));
        angles = atan2(xyPoints(:,2), xyPoints(:,1));
        
        % Sortowanie po kątach (wymagane dla lidarScan)
        [angles, idx] = sort(angles);
        ranges = ranges(idx);
        
        % Tworzenie obiektu lidarScan
        scan = lidarScan(ranges, angles);
        
        % Zapisz skan 2D, pozycję robota i czas
        lidarScans{end+1} = scan;
        robotPoses = [robotPoses; currentPose(1:3)]; % Zapisujemy x, y, i orientację
        timestamps = [timestamps; scenario.CurrentTime];
    end
    
    % Aktualizacja wizualizacji 3D
    show3D(scenario, Parent=ax, FastUpdate=true);
    waitfor(r);
    
    if ~any(isnan(currentPose))
        robotStartMoving = true;
    end
    
    % Sprawdzenie czy robot ukończył trasę lub wystąpił błąd
    if (robotStartMoving && any(isnan(currentPose))) || scenario.CurrentTime > lastInTime + 5
        disp("Symulacja zakończona. Robot ukończył trasę lub wystąpił błąd.");
        break;
    end
    
    updateSensors(scenario);
end

disp("Symulacja zakończona. Przetwarzam dane SLAM...");

%% Wizualizacja wyników

% Wizualizacja chmury punktów, jeśli są dostępne
        pcshow(pcLocs, 'blue');
        
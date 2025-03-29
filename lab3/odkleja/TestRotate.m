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

numnodes = 1000;
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
    MaxRange=10, ...
    RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ...
    ElevationResolution=1.25, ...
    AzimuthLimits=[-180 180], ... % 180° w poziomie
    ElevationLimits=[0 10], ...  % Skanowanie w zakresie 0° do 10° w pionie
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
pcArray = {};            % Tablice do przechowywania skanów chmur punktów
lidarScans = {};         % Tablice do przechowywania skanów 2D dla SLAM
timestamps = [];         % Przechowuje czasy dla każdego skanu
poseHistory = [];        % Historia pozycji robota
disp("Rozpoczynam symulację...");

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
        
        % Zapisz historię pozycji
        poseHistory = [poseHistory; position'];
        
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
            
            % Przechowuj te transformowane chmury punktów
            if ~isempty(transformedPoints)
                pcArray{end+1} = pointCloud(transformedPoints);
                
                % Tworzenie skanu 2D dla SLAM 
                % Wyodrębnij 2D z chmury punktów (ignoruj wysokość)
                points2D = transformedPoints(:, 1:2);
                
                % Przygotowanie danych do formatu lidarScan
                % Obliczenie odległości od pozycji robota
                points2D_relative = points2D - position(1:2);
                ranges = sqrt(sum(points2D_relative.^2, 2));
                
                % Obliczenie kątów (w radianach)
                angles = atan2(points2D_relative(:, 2), points2D_relative(:, 1));
                
                % Sortowanie kątów (dla poprawnego formatu skanu)
                [angles, idx] = sort(angles);
                ranges = ranges(idx);
                
                % Ograniczenie do zakresu skanera (opcjonalnie)
                maxRange = 10; % Maksymalny zasięg skanera
                validIdx = ranges <= maxRange;
                ranges = ranges(validIdx);
                angles = angles(validIdx);
                
                % Równomierne próbkowanie kątów (opcjonalnie)
                numAngles = 360; % Można dostosować rozdzielczość kątową
                anglesResampled = linspace(-pi, pi, numAngles)';
                rangesResampled = interp1(angles, ranges, anglesResampled, 'linear', maxRange);
                
                % Utworzenie obiektu lidarScan
                scan = lidarScan(rangesResampled, anglesResampled);
                lidarScans{end+1} = scan;
            end
            
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

%% Wizualizacja trajektorii robota
figure;
plot3(poseHistory(:,1), poseHistory(:,2), poseHistory(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(waypoints(:,1), waypoints(:,2), robotheight*ones(size(waypoints,1),1), 'r--', 'LineWidth', 1);
grid on;
title('Trajektoria robota vs. zaplanowana ścieżka');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Rzeczywista trajektoria', 'Zaplanowana ścieżka');

%% Wizualizacja połączonej chmury punktów
figure;
combinedPC = pccat(pcArray);
pcshow(combinedPC);
title('Połączona chmura punktów');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

%% 2D SLAM z wykorzystaniem lidarScan
try
    % Inicjalizacja SLAM
    maxLidarRange = 10; % Maksymalny zasięg LIDARa
    mapResolution = 20; % Rozdzielczość mapy w komórkach na metr
    slamAlg = lidarSLAM(mapResolution, maxLidarRange);
    
    % Konfiguracja parametrów SLAM
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 10;
    
    % Przetwarzanie wszystkich skanów LIDARa
    numScans = length(lidarScans);
    for i = 1:numScans
        if ~isempty(lidarScans{i})
            fprintf('Przetwarzanie skanu %d z %d\n', i, numScans);
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, lidarScans{i});
            if isScanAccepted
                fprintf('  Skan %d zaakceptowany\n', i);
            else
                fprintf('  Skan %d odrzucony\n', i);
            end
        end
    end
    
    % Wygenerowanie mapy
    [mapaOccupancy, posesSLAM] = buildMap(slamAlg);
    
    % Wyświetlenie wyników SLAM
    figure;
    show(slamAlg);
    title('Wyniki SLAM - mapa i graf pozycji');
    
    figure;
    show(mapaOccupancy);
    hold on;
    plot(posesSLAM(:,1), posesSLAM(:,2), 'r-', 'LineWidth', 2);
    title('Mapa zajętości i ścieżka robota');
    
catch ME
    disp('Błąd podczas wykonywania SLAM:');
    disp(ME.message);
end

%% 3D SLAM z wykorzystaniem rejestracji chmur punktów
try
    disp('Rozpoczynam 3D SLAM z rejestracją chmur punktów...');
    
    % Inicjalizacja globalnej mapy 3D
    if ~isempty(pcArray)
        globalMap = pcArray{1}; % Rozpoczynamy od pierwszego skanu
    else
        error('Brak danych chmur punktów do SLAM 3D');
    end
    
    % Parametry dla ICP (Iterative Closest Point)
    downsampleSize = 0.1;   % Rozmiar siatki do downsamplingu (m)
    maxDistance = 0.5;      % Maksymalna odległość dopasowania dla ICP (m)
    
    % Wizualizacja
    figure;
    hold on;
    title('3D SLAM z rejestracją chmur punktów');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    
    % Wyświetlenie początkowej chmury punktów
    pcshow(globalMap, 'MarkerSize', 20);
    
    % Przetwarzanie kolejnych skanów
    for i = 2:length(pcArray)
        fprintf('Rejestracja skanu %d z %d\n', i, length(pcArray));
        
        % Downsampling chmur punktów dla przyspieszenia rejestracji
        fixedPC = pcdownsample(globalMap, 'gridAverage', downsampleSize);
        movingPC = pcdownsample(pcArray{i}, 'gridAverage', downsampleSize);
        
        % Sprawdzenie, czy mamy wystarczającą liczbę punktów do rejestracji
        if fixedPC.Count < 3 || movingPC.Count < 3
            fprintf('  Pomijam skan %d z powodu niewystarczającej liczby punktów\n', i);
            continue;
        end
        
        % Użycie algorytmu ICP do dopasowania chmur punktów
        try
            tform = pcregistericp(movingPC, fixedPC, 'MaxIterations', 50, ...
                'Tolerance', [0.001, 0.001], 'MaxDistance', maxDistance);
            
            % Zastosowanie transformacji do oryginalnej chmury punktów
            alignedPC = pctransform(pcArray{i}, tform);
            
            % Połączenie dopasowanej chmury punktów z globalną mapą
            mergedPC = pcmerge(globalMap, alignedPC, 0.01); % Łączenie z rozdzielczością 1 cm
            globalMap = mergedPC;
            
            % Aktualizacja wizualizacji
            clf;
            pcshow(globalMap, 'MarkerSize', 20);
            title(sprintf('Mapa 3D po skanie %d', i));
            drawnow;
            
            fprintf('  Dodano skan %d do mapy\n', i);
        catch ME
            fprintf('  Błąd podczas przetwarzania skanu %d: %s\n', i, ME.message);
        end
    end
    
    % Końcowa wizualizacja
    figure;
    pcshow(globalMap, 'MarkerSize', 20);
    title('Końcowa mapa 3D z rejestracji chmur punktów');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    grid on;
    
    % Zapisanie wynikowej mapy do pliku
    pcwrite(globalMap, 'final_3d_map', 'PLYFormat', 'binary');
    disp('Zapisano mapę 3D do pliku final_3d_map.ply');
    
catch ME
    disp('Błąd podczas wykonywania 3D SLAM:');
    disp(ME.message);
end
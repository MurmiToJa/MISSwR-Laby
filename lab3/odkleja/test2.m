clear all;
%% Inicjalizacja scenariusza
scenario = robotScenario(UpdateRate=10);

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

numnodes = 2000;
planner = mobileRobotPRM(map, numnodes);
planner.ConnectionDistance = 1;
waypoints = findpath(planner, startPosition, goalPosition);

%% Robot
robotheight = 0.12;
numWaypoints = size(waypoints,1);
firstInTime = 0;
lastInTime = firstInTime + (numWaypoints-1);

traj = waypointTrajectory(SampleRate=5, ...
    TimeOfArrival=firstInTime:lastInTime, ...
    Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ...
    ReferenceFrame="ENU");

huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, ...
    BaseTrajectory=traj);

%% SENSOR INS
insModel = insSensor(YawAccuracy=0.1);

%% Sensor LIDAR - zaktualizowana konfiguracja 3D
lidarModel = robotLidarPointCloudGenerator(...
    UpdateRate=10, ...
    MaxRange=300, ...
    RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ...
    ElevationResolution=1.25, ...
    AzimuthLimits=[-180 180], ... % 180° w poziomie
    ElevationLimits=[0 15], ...  % Skanowanie w zakresie -15° do +15° w pionie
    HasNoise=false, ...
    HasOrganizedOutput=true);

lidar = robotSensor("lidar", platform, lidarModel, ...
    MountingLocation=[0 0 0.3], MountingAngles=[0 0 0], UpdateRate=10);
ins = robotSensor("ins", platform, insModel);

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

while advance(scenario)
    % Odczyt danych z LIDAR-a i INS
    [~, ~, currentPC] = read(lidar);
    [~, ~, insData] = read(ins);
    currentPose = read(platform);
    postition = insData.Position;

    
    if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
        disp("Valid LIDAR data found at time: " + scenario.CurrentTime);
        
        % Zapisz początkową pozycję robota, jeśli jeszcze nie jest ustawiona
        if isempty(initialPosition)
            initialPosition = currentPose(1:3);
        end
        disp(insData.Position);
        % 1. Pobranie surowej chmury punktów LIDAR-a (pominięcie NaN)
        xyzPoints = reshape(currentPC.Location, [], 3);
        validPoints = ~any(isnan(xyzPoints), 2);
        xyzPoints = xyzPoints(validPoints, :);
        
        % 2. Uwzględnienie offsetu LIDAR-a
        lidarOffset = [0 0 0.3];
        xyzPoints = xyzPoints - lidarOffset;
        
        % 3. Pobierz kąt yaw
        eul = quat2eul(insData.Orientation, 'ZYX');
        yaw = eul(1);
        %yaw = rad2deg(yaw);
        
        % 4. Najprostsze podejście - osobno rotacja, osobno translacja
        % Macierz obrotu wokół Z
        Rz = [cos(yaw), -sin(yaw), 0;
              sin(yaw), cos(yaw),  0;
              0,        0,         1];
        
        % Najpierw obrót punktów
        rotatedPoints = (Rz * xyzPoints')';
        
        % Potem translacja (przesunięcie)
        % Dodaj aktualną pozycję robota względem początku
        translatedPoints = rotatedPoints;
        translatedPoints(:,1) = translatedPoints(:,1);%+currentPose(2)-initialPosition(1);
        translatedPoints(:,2) = translatedPoints(:,2);%+currentPose(1)-initialPosition(2);
        translatedPoints(:,3) = translatedPoints(:,3);
        
        % Dodanie do globalnej chmury punktów
        globalPointCloud = pointCloud([globalPointCloud.Location; translatedPoints]);
        
                %if ~isempty(currentPC) && ~all(isnan(currentPC.Location(:))) && ~any(isnan(currentPose))
                % Wyświetl aktualną pozycję LIDARa
                % figure(2);
                % pcshow(currentPC);
                % title(['LIDAR data at time: ' num2str(scenario.CurrentTime)]);
                % xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
                % drawnow;
                % %end

        % Zapisanie pozycji robota
        robotPoses = [robotPoses; currentPose(1:3) - initialPosition];
        timestamps = [timestamps; scenario.CurrentTime];
        
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

    updateSensors(scenario);
    advance(scenario);
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

%% 
%pcshow(currentPC)
% % Wyświetlenie wartości orientacji
% disp('Orientacja:');
% disp(insData.Orientation);
% 
% % Sprawdzenie, czy zmienna jest kwaternionem
% if isa(insData.Orientation, 'quaternion')
%     % Konwersja kwaternionu na macierz obrotu
%     rotMat = quat2rotm(insData.Orientation);
%     disp('Macierz obrotu:');
%     disp(rotMat);
% 
%     % Konwersja kwaternionu na kąty Eulera (w radianach, ewentualnie przelicz na stopnie)
%     eul = quat2eul(insData.Orientation);
%     disp('Kąty Eulera [roll, pitch, yaw] (w radianach):');
%     disp(eul);
% 
%     % Jeśli potrzebujesz stopni, możesz przeliczyć:
%     eulDeg = rad2deg(eul);
%     disp('Kąty Eulera [roll, pitch, yaw] (w stopniach):');
%     disp(eulDeg);
% else
%     disp('Orientacja nie jest obiektem typu quaternion.');
% end

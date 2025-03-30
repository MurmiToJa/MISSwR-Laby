clc; clear all; close all;

%% Wczytanie danych
load('simulation_data.mat');
disp('Dane wczytane pomyślnie. Rozpoczynam SLAM...');

%% Przygotowanie danych do SLAM
maxLidarRange = 10;
mapResolution = 20;
scans2D = {};

for i = 1:length(simData)
    if ~isempty(simData(i).scans)
        xyzPoints = simData(i).scans.Location;
        xyzPoints = reshape(xyzPoints, [], 3);
        validPoints = ~any(isnan(xyzPoints), 2) & all(xyzPoints >= 0, 2);
        xyPoints = xyzPoints(validPoints, 1:2);
        
        if ~isempty(xyPoints)
            [theta, rho] = cart2pol(xyPoints(:,1), xyPoints(:,2));
            valid = rho > 0.1 & rho <= maxLidarRange;
            scans2D{end+1} = lidarScan(rho(valid), theta(valid));
        end
    end
end

%% Inicjalizacja i przetwarzanie SLAM
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;
slamAlg.LoopClosureSearchRadius = 8;

figure('Name', 'SLAM Processing', 'NumberTitle', 'off');
for i = 1:length(scans2D)
    [isScanAccepted, ~, ~] = addScan(slamAlg, scans2D{i});
    if mod(i,10) == 0
        show(slamAlg); drawnow;
        fprintf('Przetworzono %d/%d skanów\n', i, length(scans2D));
    end
end

%% Finalizacja mapy
optimizePoseGraph(slamAlg);
[scans, optimizedPoses] = scansAndPoses(slamAlg);
occupancyMap = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

%% Wizualizacja wyników
figure('Name', 'Wyniki SLAM', 'NumberTitle', 'off');
subplot(1,2,1); show(occupancyMap); title('Mapa zajętości');
subplot(1,2,2); show(slamAlg.PoseGraph); title('Graf pozycji');

disp('SLAM zakończony pomyślnie!');
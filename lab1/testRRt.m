%% Inicjalizacja
clc;
clear all;
close all;
%% Parametry
start_point = [20, 50];
end_point = [170, 50];
start = [start_point 0]; % Punkt startowy z orientacją
goal = [end_point 0]; % Punkt końcowy z orientacją
block_size = 5; % Rozmiar przeszkód
num_obstacles = 10; % Liczba przeszkód do wygenerowania
MaxConnectionDistance = [1, 5, 10]; %MaxConnectionDistance
colors = ["b", "g", "m"]; % Kolory dla różnych liczby węzłów
pathColors = ["r", "c", "y"]; % Kolory dla głównej ścieżki na każdej figurze
%% Wczytanie i przetworzenie mapy
mapa = imread('mapa2.jpg');
if size(mapa, 3) > 1
    mapa = rgb2gray(mapa);
end
% Binaryzacja mapy - ściany są czarne (0), wolna przestrzeń jest biała (1)
binMapa = mapa > 128;
[height, width] = size(binMapa);
% Aktualizacja mapy o przeszkody
% Tworzenie mapy zajętości dla nawigacji
mapOccupancy = binaryOccupancyMap(~binMapa);
map = mapOccupancy;
% Przekazujemy pełne wektory start i goal zamiast tylko punktów, plus wymiary mapy
rrtPathPlanning(mapOccupancy, start, goal, MaxConnectionDistance, colors, pathColors);
%% Planowanie ścieżki przy użyciu RRT
function rrtPathPlanning(map, start, goal, ValidationDistance, colors, pathColors)
    start_point = start(1:2); % Wyciągamy współrzędne punktu startowego
    end_point = goal(1:2); % Wyciągamy współrzędne punktu końcowego
    
    for idx = 1:length(ValidationDistance)
        figure;
        sv.ValidationDistance = ValidationDistance(idx);
        ss = stateSpaceSE2;
        %MaxConnectionDistance
        Validation_Distance = sv.ValidationDistance;
        ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        sv = validatorOccupancyMap(ss, Map=map);
        planner = plannerRRT(ss, sv, "MaxConnectionDistance",Validation_Distance);
        %planner.MaxNumTreeNodes = numNodes; % Ustawienie maksymalnej liczby węzłów
        
        rng(100, 'twister'); % dla powtarzalnych wyników
        
        % Mierzenie czasu planowania
        tic;
        [pthObj, solnInfo] = plan(planner, start, goal);
        planningTime = toc;
        
        % Liczba faktycznie użytych węzłów
        actualNodes = size(solnInfo.TreeData, 1);
        
        show(map)
        hold on
        
        % Rysowanie drzewa ekspansji
        plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', 'Color', colors(idx))
        
        % Rysowanie punktu startowego i końcowego
        plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2)
        plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2)
        
        % Rysowanie ścieżki jeśli znaleziono
        if solnInfo.IsPathFound
            interpolate(pthObj, 2320);
            plot(pthObj.States(:,1), pthObj.States(:,2), ...
                "Color", pathColors(idx), "LineWidth", 2.5);
            titleInfo = sprintf('RRT z ValidationDistance = %d  (użyto węzłów: %d, czas: %.2f s)', ValidationDistance(idx), actualNodes, planningTime);
        else
            disp("Nie znaleziono ścieżki dla ValidationDistance = " + ValidationDistance(idx));
            titleInfo = sprintf('RRT brak ścieżki');
        end
        
        title(titleInfo);
        hold off
    end
end
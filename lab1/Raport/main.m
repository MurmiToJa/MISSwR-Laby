%% Inicjalizacja
clc;
clear all;
close all;

%% Parametry
% pierwsza mapa
% start_point = [3, 13];
% end_point = [13, 3];
% MaxConnectionDistance = [0.5,1];

%druga mapa
% start_point = [3, 13];
% end_point = [13, 3];
% MaxConnectionDistance = [1,2,3];


%trzecia mapa
start_point = [3, 2];
end_point = [28, 27];
MaxConnectionDistance = [1,2,3];



start = [start_point 0]; % Punkt startowy z orientacją
goal = [end_point 0];    % Punkt końcowy z orientacją
block_size = 5;          % Rozmiar przeszkód
num_obstacles = [0,5,10];       % Liczba przeszkód do wygenerowania
numNodesList = [0,0,0]; % Liczby węzłów do testowania
colors = ["b", "g", "m"];       % Kolory dla różnych liczby węzłów
pathColors = ["r", "c", "y"];   % Kolory dla głównej ścieżki na każdej figurze

%% Wczytanie i przetworzenie mapy
% mapa = imread('mapa1.jpg');
mapa = imread('mapa3.jpg');
% mapa = imread('mapa3.jpg');
if size(mapa, 3) > 1
    mapa = rgb2gray(mapa);
end

% Binaryzacja mapy - ściany są czarne (0), wolna przestrzeń jest biała (1)
binMapa = mapa > 128;
[height, width] = size(binMapa);

%% Generowanie przeszkód
obstacles = generateObstacles(binMapa, num_obstacles, block_size, start_point, end_point, width, height);

% Aktualizacja mapy o przeszkody
binMapa = updateMapWithObstacles(binMapa, obstacles, block_size);

% Tworzenie mapy zajętości dla nawigacji
mapOccupancy = binaryOccupancyMap(~binMapa);

% Wyświetlenie mapy z przeszkodami
displayMap(mapOccupancy, start_point, end_point);

%% Planowanie ścieżki przy użyciu RRT
%rrtPathPlanning(mapOccupancy, start, goal, MaxConnectionDistance, colors, pathColors);
numNodesList = rrtPathPlanning(mapOccupancy, start, goal, MaxConnectionDistance, colors, pathColors, numNodesList);
disp(numNodesList);

%% Planowanie ścieżki przy użyciu PRM z różnymi liczbami węzłów
planPathWithPRM(mapOccupancy, start, goal, numNodesList, colors, pathColors);

%% Funkcje pomocnicze

% Funkcja generująca losowe przeszkody
function obstacles = generateObstacles(binMapa, num_obstacles, block_size, start_point, end_point, width, height)
    obstacles = zeros(num_obstacles, 2);
    count = 0;
    max_attempts = 1000;
    attempts = 0;
    
    while count < num_obstacles && attempts < max_attempts
        % Losowy punkt na mapie
        x = randi([1, width - block_size]);
        y = randi([1, height - block_size]);
        
        % Sprawdź warunki kolizji
        wall_collision = checkWallCollision(binMapa, x, y, block_size);
        start_collision = checkPointCollision(x, y, block_size, start_point(1), start_point(2));
        end_collision = checkPointCollision(x, y, block_size, end_point(1), end_point(2));
        
        block_collision = false;
        if count > 0
            block_collision = checkBlockCollision(obstacles(1:count, :), x, y, block_size);
        end
        
        % Jeśli brak kolizji, dodaj przeszkodę
        if ~wall_collision && ~start_collision && ~end_collision && ~block_collision
            count = count + 1;
            obstacles(count, :) = [x, y];
        end
        
        attempts = attempts + 1;
    end
    
    % Jeśli nie udało się wygenerować wszystkich przeszkód, przytnij tablicę
    obstacles = obstacles(1:count, :);
end

% Funkcja sprawdzająca kolizję ze ścianami
function collision = checkWallCollision(binMapa, x, y, block_size)
    [height, width] = size(binMapa);
    
    % Sprawdź czy blok jest w granicach mapy
    if x < 1 || y < 1 || x+block_size-1 > width || y+block_size-1 > height
        collision = true;
        return;
    end
    
    % Sprawdź czy blok nie koliduje ze ścianami (wartości 0 w binMapa)
    block_area = binMapa(y:y+block_size-1, x:x+block_size-1);
    if any(block_area(:) == 0)
        collision = true;
        return;
    end
    
    % Sprawdź czy blok nie jest przy ścianie (sprawdź otoczenie bloku)
    % Górna krawędź
    if y > 1 && any(binMapa(y-1, x:x+block_size-1) == 0)
        collision = true;
        return;
    end
    
    % Dolna krawędź
    if y+block_size <= height && any(binMapa(y+block_size, x:x+block_size-1) == 0)
        collision = true;
        return;
    end
    
    % Lewa krawędź
    if x > 1 && any(binMapa(y:y+block_size-1, x-1) == 0)
        collision = true;
        return;
    end
    
    % Prawa krawędź
    if x+block_size <= width && any(binMapa(y:y+block_size-1, x+block_size) == 0)
        collision = true;
        return;
    end
    
    collision = false;
end

% Funkcja sprawdzająca kolizję z punktami
function collision = checkPointCollision(block_x, block_y, block_size, point_x, point_y)
    collision = (point_x >= block_x && point_x < block_x + block_size && ...
                 point_y >= block_y && point_y < block_y + block_size);
end

% Funkcja sprawdzająca kolizję z innymi blokami
function collision = checkBlockCollision(obstacles, new_block_x, new_block_y, block_size)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        
        % Sprawdź czy bloki się nakładają
        if new_block_x < obs_x + block_size && new_block_x + block_size > obs_x && ...
           new_block_y < obs_y + block_size && new_block_y + block_size > obs_y
            collision = true;
            return;
        end
    end
end

% Funkcja aktualizująca mapę o przeszkody
function updatedMap = updateMapWithObstacles(binMapa, obstacles, block_size)
    updatedMap = binMapa;
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1);
        y = obstacles(i, 2);
        updatedMap(y:y+block_size-1, x:x+block_size-1) = 0; % Dodaj przeszkodę (wartość 0)
    end
end

% Funkcja wyświetlająca mapę
function displayMap(mapOccupancy, start_point, end_point)
    figure;
    show(mapOccupancy);
    hold on;
    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Punkt startowy
    plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Punkt końcowy
    title('Mapa z przeszkodami');
    hold off;
end

% Funkcja planowania ścieżki przy użyciu PRM
function planPathWithPRM(map, start, goal, numNodesList, colors, pathColors)
    ss = stateSpaceSE2;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv = validatorOccupancyMap(ss, Map=map);
    sv.ValidationDistance = 0.01;
    
    for idx = 1:length(numNodesList)
        maxNodes = numNodesList(idx);
        disp(maxNodes);
        planner = plannerPRM(ss, sv, "MaxNumNodes",maxNodes);
        
        % Pobranie danych grafu
        graph = graphData(planner);
        edges = table2array(graph.Edges);
        nodes = table2array(graph.Nodes);
        
        % Nowa figura dla każdego przypadku
        figure;
        show(sv.Map)
        hold on
        
        % Rysowanie węzłów
        plot(nodes(:,1), nodes(:,2), "*", "Color", colors(idx), "LineWidth", 2);
        
        % Rysowanie krawędzi
        for i = 1:size(edges,1)
            states = interpolate(ss, nodes(edges(i,1),:), ...
                nodes(edges(i,2),:), 0:0.02:1);
            plot(states(:,1), states(:,2), "Color", colors(idx))
        end
        
        % Planowanie ścieżki i pomiar czasu
        rng(100,"twister");
        tic; % Rozpoczęcie pomiaru czasu
        [pthObj, solnInfo] = plan(planner, start, goal);
        
        
        if solnInfo.IsPathFound
            interpolate(pthObj, 1000);
            plot(pthObj.States(:,1), pthObj.States(:,2), ...
                "Color", pathColors(idx), "LineWidth", 2.5);
            planTime = toc;
            timeText = sprintf('Czas planowania: %.4f s', planTime);
        else
            disp("Nie znaleziono ścieżki dla MaxNumNodes = " + maxNodes);
            planTime = toc;
            timeText = sprintf('Czas planowania: %.4f s (ścieżka nie znaleziona)', planTime);
        end
        
        % Rysowanie punktu startowego i końcowego
        plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3)
        plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3)
        
        % Dodanie czasu planowania do tytułu
        title(sprintf("PRM z MaxNumNodes = %d\n%s", maxNodes, timeText));
        
        hold off;
    end
end

%% Funkcja planowania ścieżki przy użyciu RRT
%jeszcze do poprawy
function numNodesList = rrtPathPlanning(map, start, goal, ValidationDistance, colors, pathColors,numNodesList)
    start_point = start(1:2);
    end_point = goal(1:2);
    
    for idy = 1:length(ValidationDistance)
        figure;
        ss = stateSpaceSE2;
        ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        sv = validatorOccupancyMap(ss, Map=map);
        sv.ValidationDistance = 0.1;
        Max_Validation_Distance = ValidationDistance(idy);

        % figure;
        % sv.ValidationDistance = ValidationDistance(idy);
        % ss = stateSpaceSE2;
        % %MaxConnectionDistance
        % Validation_Distance = sv.ValidationDistance;
        % 
        % ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        % sv = validatorOccupancyMap(ss, Map=map);
        planner = plannerRRT(ss, sv, "MaxConnectionDistance",Max_Validation_Distance);
        %planner.MaxNumTreeNodes = numNodes; % Ustawienie maksymalnej liczby węzłów
        
        rng(100, 'twister'); % dla powtarzalnych wyników
        
        % Mierzenie czasu planowania
        tic;
        [pthObj, solnInfo] = plan(planner, start, goal);
        
        
        % Liczba faktycznie użytych węzłów
        actualNodes = size(solnInfo.TreeData, 1);
        numNodesList(idy) = actualNodes;

        show(map)
        hold on
        
        % Rysowanie drzewa ekspansji
        plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', 'Color', colors(idy))
        
        % Rysowanie punktu startowego i końcowego
        plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2)
        plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2)
        
        % Rysowanie ścieżki jeśli znaleziono
        if solnInfo.IsPathFound
            interpolate(pthObj, 2320);
            plot(pthObj.States(:,1), pthObj.States(:,2), ...
                "Color", pathColors(idy), "LineWidth", 2.5);
            planningTime = toc;
            titleInfo = sprintf('RRT z ValidationDistance = %d  (użyto węzłów: %d, czas: %.2f s)', ValidationDistance(idy), actualNodes, planningTime);
        else
            planningTime = toc;
            disp("Nie znaleziono ścieżki dla ValidationDistance = " + ValidationDistance(idy));
            titleInfo = sprintf('RRT brak ścieżki');
        end
        title(titleInfo);
        hold off
    end
end
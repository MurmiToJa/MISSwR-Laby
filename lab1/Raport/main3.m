%% Inicjalizacja
clc;
clear all;
close all;

% Uruchomienie głównej funkcji
runAllMaps();

%% Funkcja uruchamiająca analizę dla wszystkich map
function runAllMaps()
    % Utwórz główny folder dla wyników
    mainFolder = 'wyniki';
    if ~exist(mainFolder, 'dir')
        mkdir(mainFolder);
    end
    
    % Utwórz plik CSV do zapisu statystyk
    csvFile = fullfile(mainFolder, 'statystyki.csv');
    fid = fopen(csvFile, 'w');
    % Nagłówek CSV
    fprintf(fid, 'Mapa,Metoda,Liczba przeszkód,Czas planowania,Liczba węzłów,MaxConnectionDistance/MaxNumNodes,Znaleziono ścieżkę\n');
    
    % Lista map do przetworzenia
    mapFiles = {'mapa1.jpg', 'mapa2.jpg', 'mapa3.jpg'};
    
    % Parametry dla różnych map
    params = struct();
    
    % Mapa 1
    params(1).start_point = [3, 13];
    params(1).end_point = [13, 3];
    params(1).MaxConnectionDistance = [0.5, 1];
    params(1).num_obstacles_list = [0, 5, 10];
    
    % Mapa 2
    params(2).start_point = [3, 13];
    params(2).end_point = [13, 3];
    params(2).MaxConnectionDistance = [1, 2, 3];
    params(2).num_obstacles_list = [0, 5, 10];
    
    % Mapa 3
    params(3).start_point = [3, 2];
    params(3).end_point = [28, 27];
    params(3).MaxConnectionDistance = [1, 2, 3];
    params(3).num_obstacles_list = [0, 5, 10];
    
    % Definicja kolorów
    colors = ["b", "g", "m"];
    pathColors = ["r", "c", "y"];
    
    % Przetwarzanie każdej mapy
    for mapIdx = 1:length(mapFiles)
        mapFile = mapFiles{mapIdx};
        processMap(mapFile, params(mapIdx), colors, pathColors, mainFolder, fid, mapIdx);
    end
    
    % Zamknij plik CSV
    fclose(fid);
    disp('Zakończono przetwarzanie wszystkich map!');
end

%% Funkcja przetwarzająca pojedynczą mapę
function processMap(mapFile, params, colors, pathColors, mainFolder, fid, mapIdx)
    % Stwórz folder dla tej mapy
    mapFolder = fullfile(mainFolder, ['mapa' num2str(mapIdx)]);
    if ~exist(mapFolder, 'dir')
        mkdir(mapFolder);
    end
    
    % Stwórz foldery dla metod
    rrtFolder = fullfile(mapFolder, 'RRT');
    prmFolder = fullfile(mapFolder, 'PRM');
    if ~exist(rrtFolder, 'dir')
        mkdir(rrtFolder);
    end
    if ~exist(prmFolder, 'dir')
        mkdir(prmFolder);
    end
    
    % Parametry
    start_point = params.start_point;
    end_point = params.end_point;
    MaxConnectionDistance = params.MaxConnectionDistance;
    num_obstacles_list = params.num_obstacles_list;
    
    % Wczytanie i przetworzenie mapy
    mapa = imread(mapFile);
    if size(mapa, 3) > 1
        mapa = rgb2gray(mapa);
    end
    
    % Binaryzacja mapy - ściany są czarne (0), wolna przestrzeń jest biała (1)
    binMapa = mapa > 128;
    [height, width] = size(binMapa);
    block_size = 1; % Rozmiar przeszkód
    
    % Dla każdej liczby przeszkód
    for obstIdx = 1:length(num_obstacles_list)
        num_obstacles = num_obstacles_list(obstIdx);
        
        % Stwórz foldery dla różnej liczby przeszkód
        rrtObsFolder = fullfile(rrtFolder, ['przeszkody_' num2str(num_obstacles)]);
        prmObsFolder = fullfile(prmFolder, ['przeszkody_' num2str(num_obstacles)]);
        if ~exist(rrtObsFolder, 'dir')
            mkdir(rrtObsFolder);
        end
        if ~exist(prmObsFolder, 'dir')
            mkdir(prmObsFolder);
        end
        
        % Generowanie przeszkód - upewniamy się, że num_obstacles jest skalarem
        obstacles = generateObstacles(binMapa, double(num_obstacles), block_size, start_point, end_point, width, height);
        
        % Aktualizacja mapy o przeszkody
        updatedBinMapa = updateMapWithObstacles(binMapa, obstacles, block_size);
        
        % Tworzenie mapy zajętości dla nawigacji
        mapOccupancy = binaryOccupancyMap(~updatedBinMapa);
        
        % Wyświetlenie mapy z przeszkodami
        figMap = figure('Visible', 'off');
        show(mapOccupancy);
        hold on;
        plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Punkt startowy
        plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Punkt końcowy
        title(['Mapa ' num2str(mapIdx) ' z ' num2str(num_obstacles) ' przeszkodami']);
        mapImagePath = fullfile(mapFolder, ['mapa' num2str(mapIdx) '_' num2str(num_obstacles) 'przeszkod.png']);
        saveas(figMap, mapImagePath);
        close(figMap);
        
        % Planowanie ścieżki przy użyciu RRT
        start = [start_point 0]; % Punkt startowy z orientacją
        goal = [end_point 0];    % Punkt końcowy z orientacją
        
        % Uruchom RRT i zapisz liczbę węzłów
        numNodesList = rrtPathPlanning(mapOccupancy, start, goal, MaxConnectionDistance, colors, pathColors, rrtObsFolder, fid, mapIdx, num_obstacles);
        
        % Planowanie ścieżki przy użyciu PRM z różnymi liczbami węzłów
        planPathWithPRM(mapOccupancy, start, goal, numNodesList, colors, pathColors, prmObsFolder, fid, mapIdx, num_obstacles);
    end
end

%% Planowanie ścieżki przy użyciu RRT
function numNodesList = rrtPathPlanning(map, start, goal, ValidationDistance, colors, pathColors, outputFolder, fid, mapIdx, num_obstacles)
    start_point = start(1:2);
    end_point = goal(1:2);
    numNodesList = zeros(1, length(ValidationDistance));
    
    for idy = 1:length(ValidationDistance)
        figRRT = figure('Visible', 'off');
        ss = stateSpaceSE2;
        ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        sv = validatorOccupancyMap(ss, Map=map);
        sv.ValidationDistance = 0.1;
        Max_Validation_Distance = ValidationDistance(idy);

        planner = plannerRRT(ss, sv, "MaxConnectionDistance", Max_Validation_Distance);
        
        rng(100, 'twister'); % dla powtarzalnych wyników
        
        % Mierzenie czasu planowania
        tic;
        [pthObj, solnInfo] = plan(planner, start, goal);
        planningTime = toc;
        
        % Liczba faktycznie użytych węzłów
        actualNodes = size(solnInfo.TreeData, 1);
        numNodesList(idy) = actualNodes;

        show(map);
        hold on;
        
        % Rysowanie drzewa ekspansji
        plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', 'Color', colors(idy));
        
        % Rysowanie punktu startowego i końcowego
        plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
        plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        
        % Rysowanie ścieżki jeśli znaleziono
        pathFound = solnInfo.IsPathFound;
        if pathFound
            interpolate(pthObj, 2320);
            plot(pthObj.States(:,1), pthObj.States(:,2), ...
                "Color", pathColors(idy), "LineWidth", 2.5);
            titleInfo = sprintf('RRT z ValidationDistance = %g (użyto węzłów: %d, czas: %.2f s)', ValidationDistance(idy), actualNodes, planningTime);
        else
            titleInfo = sprintf('RRT z ValidationDistance = %g (brak ścieżki, czas: %.2f s)', ValidationDistance(idy), planningTime);
        end
        title(titleInfo);
        
        % Zapisz rysunek
        figName = sprintf('RRT_ValidationDistance_%g_przeszkody_%d.png', ValidationDistance(idy), num_obstacles);
        saveas(figRRT, fullfile(outputFolder, figName));
        close(figRRT);
        
        % Zapisz statystyki do CSV
        fprintf(fid, '%d,RRT,%d,%f,%d,%g,%d\n', mapIdx, num_obstacles, planningTime, actualNodes, ValidationDistance(idy), pathFound);
    end
end

%% Funkcja planowania ścieżki przy użyciu PRM
function planPathWithPRM(map, start, goal, numNodesList, colors, pathColors, outputFolder, fid, mapIdx, num_obstacles)
    ss = stateSpaceSE2;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv = validatorOccupancyMap(ss, Map=map);
    sv.ValidationDistance = 0.01;
    
    for idx = 1:length(numNodesList)
        % Ustal minimum 10 węzłów
        maxNodes = max(numNodesList(idx), 10);
        
        figPRM = figure('Visible', 'off');
        planner = plannerPRM(ss, sv, "MaxNumNodes", maxNodes);
        
        % Pobranie danych grafu
        try
            graph = graphData(planner);
            edges = table2array(graph.Edges);
            nodes = table2array(graph.Nodes);
            
            show(sv.Map);
            hold on;
            
            % Rysowanie węzłów
            plot(nodes(:,1), nodes(:,2), "*", "Color", colors(idx), "LineWidth", 2);
            
            % Rysowanie krawędzi
            for i = 1:size(edges,1)
                states = interpolate(ss, nodes(edges(i,1),:), ...
                    nodes(edges(i,2),:), 0:0.02:1);
                plot(states(:,1), states(:,2), "Color", colors(idx));
            end
            
            % Planowanie ścieżki i pomiar czasu
            rng(100, "twister");
            tic; % Rozpoczęcie pomiaru czasu
            [pthObj, solnInfo] = plan(planner, start, goal);
            planTime = toc;
            
            pathFound = solnInfo.IsPathFound;
            if pathFound
                interpolate(pthObj, 1000);
                plot(pthObj.States(:,1), pthObj.States(:,2), ...
                    "Color", pathColors(idx), "LineWidth", 2.5);
                timeText = sprintf('Czas planowania: %.4f s', planTime);
            else
                timeText = sprintf('Czas planowania: %.4f s (ścieżka nie znaleziona)', planTime);
            end
            
            % Rysowanie punktu startowego i końcowego
            plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3);
            plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3);
            
            % Dodanie czasu planowania do tytułu
            title(sprintf("PRM z MaxNumNodes = %d\n%s", maxNodes, timeText));
            
            % Zapisz rysunek
            figName = sprintf('PRM_MaxNumNodes_%d_przeszkody_%d.png', maxNodes, num_obstacles);
            saveas(figPRM, fullfile(outputFolder, figName));
            
            % Zapisz statystyki do CSV
            fprintf(fid, '%d,PRM,%d,%f,%d,%d,%d\n', mapIdx, num_obstacles, planTime, size(nodes, 1), maxNodes, pathFound);
            
        catch ME
            warning('Błąd podczas planowania PRM: %s', ME.message);
            % Zapisz informację o błędzie do CSV
            fprintf(fid, '%d,PRM,%d,0,0,%d,0\n', mapIdx, num_obstacles, maxNodes);
        end
        close(figPRM);
    end
end

%% Funkcje pomocnicze

% Funkcja generująca losowe przeszkody
function obstacles = generateObstacles(binMapa, num_obstacles, block_size, start_point, end_point, width, height)
    % Upewnij się, że num_obstacles jest skalarem
    if ~isscalar(num_obstacles)
        num_obstacles = num_obstacles(1);
    end
    
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
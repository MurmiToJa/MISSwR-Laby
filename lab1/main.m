%% wyczyszczenie
clc;
clear all;

%% Wczytanie mapy
mapa = imread('mapa2.jpg');
if size(mapa, 3) > 1
    mapa = rgb2gray(mapa);
end

% Binearyzacja mapy - ściany są czarne (0), wolna przestrzeń jest biała (1)
binMapa = mapa > 128; % Próg binaryzacji

% Definicja punktu początkowego i końcowego
start_point = [10, 10];
end_point = [180, 10];

start = [10 10 0];
goal = [180 10 0];

% Parametry generowania przeszkód
block_size = 5;
num_obstacles = 0; % liczba przeszkód do wygenerowania
[height, width] = size(binMapa);

% Funkcja sprawdzająca czy blok koliduje ze ścianami
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
    if y > 1
        top_edge = binMapa(y-1, x:x+block_size-1);
        if any(top_edge == 0)
            collision = true;
            return;
        end
    end
    
    % Dolna krawędź
    if y+block_size <= height
        bottom_edge = binMapa(y+block_size, x:x+block_size-1);
        if any(bottom_edge == 0)
            collision = true;
            return;
        end
    end
    
    % Lewa krawędź
    if x > 1
        left_edge = binMapa(y:y+block_size-1, x-1);
        if any(left_edge == 0)
            collision = true;
            return;
        end
    end
    
    % Prawa krawędź
    if x+block_size <= width
        right_edge = binMapa(y:y+block_size-1, x+block_size);
        if any(right_edge == 0)
            collision = true;
            return;
        end
    end
    
    collision = false;
end

% Funkcja sprawdzająca czy blok koliduje z punktami startu/końca
function collision = checkPointCollision(block_x, block_y, block_size, point_x, point_y)
    if point_x >= block_x && point_x < block_x + block_size && ...
       point_y >= block_y && point_y < block_y + block_size
        collision = true;
    else
        collision = false;
    end
end

% Funkcja sprawdzająca czy blok koliduje z innymi blokami
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

% Generowanie przeszkód
obstacles = zeros(num_obstacles, 2); % [x, y] dla każdej przeszkody
count = 0;
max_attempts = 1000; % maksymalna liczba prób wygenerowania przeszkody
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

% Aktualizacja mapy o przeszkody
for i = 1:count
    x = obstacles(i, 1);
    y = obstacles(i, 2);
    binMapa(y:y+block_size-1, x:x+block_size-1) = 0; % Dodaj przeszkodę (wartość 0)
end

% Tworzenie mapy zajętości dla nawigacji
mapOccupancy = binaryOccupancyMap(~binMapa);

% Wyświetlenie mapy z przeszkodami
figure;
show(mapOccupancy);
hold on;
plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Punkt startowy
plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Punkt końcowy
title('Mapa z losowymi przeszkodami');
hold off;

%% PRM
map = mapOccupancy;
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss,Map=map);
sv.ValidationDistance = 0.01;
numNodesList = [10, 100, 1000];
colors = ["b", "g", "m"]; % Kolory dla różnych liczby nodów
pathColors = ["r", "c", "y"]; % Kolory dla głównej ścieżki na każdej figurze

for idx = 1:length(numNodesList)
    maxNodes = numNodesList(idx);
    planner = plannerPRM(ss, sv, "MaxNumNodes", maxNodes);

    % Pobranie danych grafu
    graph = graphData(planner);
    edges = table2array(graph.Edges);
    nodes = table2array(graph.Nodes);

    % Nowa figura dla każdego przypadku
    figure;
    show(sv.Map)
    hold on

    % Rysowanie nodów
    plot(nodes(:,1), nodes(:,2), "*", "Color", colors(idx), "LineWidth", 2);

    % Rysowanie krawędzi
    for i = 1:size(edges,1)
        states = interpolate(ss, nodes(edges(i,1),:), ...
                             nodes(edges(i,2),:), 0:0.02:1);
        plot(states(:,1), states(:,2), "Color", colors(idx))
    end

    % Planowanie ścieżki
    rng(100,"twister");
    [pthObj, solnInfo] = plan(planner, start, goal);
    if solnInfo.IsPathFound
        interpolate(pthObj, 1000);
        plot(pthObj.States(:,1), pthObj.States(:,2), ...
             "Color", pathColors(idx), "LineWidth", 2.5);
    else
        disp("Path not found for " + maxNodes);
    end

    % Rysowanie punktu startowego i końcowego
    plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3)
    plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3)

    title(sprintf("PRM z MaxNumNodes = %d", maxNodes));
    hold off;
end

%% PRM
%numnodes
for idy = 1:length(numNodesList)
    nodes = numNodesList(idy);
    prmSimple = mobileRobotPRM(map,nodes);
    startLocation = [10 10];
    endLocation = [12 10];
    path = findpath(prmSimple,start_point,end_point);
    show(prmSimple)
end
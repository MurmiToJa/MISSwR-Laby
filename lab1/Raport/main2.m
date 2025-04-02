%% Inicjalizacja
clc;
clear all;
close all;

%% Parametry ilości przeszkód
obstacle_counts = [0, 5, 10]; % Trzy różne ilości przeszkód

%% Przetwarzanie każdej mapy
for map_idx = 1:3
    % Ustaw parametry w zależności od mapy
    if map_idx == 1
        start_point = [3, 13];
        end_point = [13, 3];
        MaxConnectionDistance = [0.5, 1];
        mapa_path = 'mapa1.jpg';
    elseif map_idx == 2
        start_point = [3, 13];
        end_point = [13, 3];
        MaxConnectionDistance = [1, 2, 3];
        mapa_path = 'mapa2.jpg';
    else % map_idx == 3
        start_point = [3, 2];
        end_point = [28, 27];
        MaxConnectionDistance = [3, 4, 5];
        mapa_path = 'mapa3.jpg';
    end
    
    % Stwórz folder główny dla mapy
    base_folder = sprintf('mapa%d', map_idx);
    if ~exist(base_folder, 'dir')
        mkdir(base_folder);
    end
    
    % Dla każdej ilości przeszkód
    for obs_idx = 1:length(obstacle_counts)
        num_obstacles = obstacle_counts(obs_idx);
        
        % Utwórz podfolder dla określonej liczby przeszkód
        obstacles_folder = fullfile(base_folder, sprintf('obstacles_%d', num_obstacles));
        if ~exist(obstacles_folder, 'dir')
            mkdir(obstacles_folder);
        end
        
        % Utwórz podfoldery dla metod
        rrt_folder = fullfile(obstacles_folder, 'RRT');
        prm_folder = fullfile(obstacles_folder, 'PRM');
        
        if ~exist(rrt_folder, 'dir')
            mkdir(rrt_folder);
        end
        
        if ~exist(prm_folder, 'dir')
            mkdir(prm_folder);
        end
        
        % Pozostałe parametry
        start = [start_point 0]; % Punkt startowy z orientacją
        goal = [end_point 0];    % Punkt końcowy z orientacją
        block_size = 5;          % Rozmiar przeszkód
        
        % Inicjalizuj numNodesList z odpowiednią długością
        numNodesList = zeros(1, length(MaxConnectionDistance));
        colors = ["b", "g", "m"];       % Kolory dla różnych liczby węzłów
        pathColors = ["r", "c", "y"];   % Kolory dla głównej ścieżki na każdej figurze
        
        %% Wczytanie i przetworzenie mapy
        mapa = imread(mapa_path);
        if size(mapa, 3) > 1
            mapa = rgb2gray(mapa);
        end
        
        % Binaryzacja mapy - ściany są czarne (0), wolna przestrzeń jest biała (1)
        binMapa = mapa > 128;
        [height, width] = size(binMapa);
        
        %% Generowanie przeszkód - POPRAWIONE
        disp(['Generowanie ' num2str(num_obstacles) ' przeszkód dla mapy ' num2str(map_idx)]);
        
        if num_obstacles > 0
            % Zmodyfikuj generateObstacles aby zwiększyć max_attempts i dodać debug
            obstacles = generateObstacles(binMapa, num_obstacles, block_size, start_point, end_point, width, height);
            
            % Sprawdź faktyczną liczbę wygenerowanych przeszkód
            actual_obstacles = size(obstacles, 1);
            disp(['Faktycznie wygenerowano ' num2str(actual_obstacles) ' z ' num2str(num_obstacles) ' przeszkód']);
            
            % Aktualizacja mapy o przeszkody
            binMapa = updateMapWithObstacles(binMapa, obstacles, block_size);
        else
            disp('Brak przeszkód do wygenerowania');
            obstacles = zeros(0, 2); % Pusta tablica
        end
        
        % Tworzenie mapy zajętości dla nawigacji
        mapOccupancy = binaryOccupancyMap(~binMapa);
        
        % Wyświetlenie mapy z przeszkodami

        
        %% Planowanie ścieżki przy użyciu RRT
        % Wywołaj funkcję RRT i zapisz figury
        numNodesList = rrtPathPlanning(mapOccupancy, start, goal, MaxConnectionDistance, colors, pathColors, numNodesList, rrt_folder);
        disp(['Liczba węzłów dla mapy ', num2str(map_idx), ' z ', num2str(num_obstacles), ' przeszkodami: ']);
        disp(numNodesList);
        
        %% Planowanie ścieżki przy użyciu PRM z różnymi liczbami węzłów
        % Zapewnij, że wszystkie wartości numNodesList są większe od zera
        for i = 1:length(numNodesList)
            if numNodesList(i) <= 0
                numNodesList(i) = 100; % Domyślna wartość jeśli mamy 0
            end
        end
        
        % Wywołaj funkcję PRM i zapisz figury
        planPathWithPRM(mapOccupancy, start, goal, numNodesList, colors(1:length(numNodesList)), pathColors(1:length(numNodesList)), prm_folder);
        
        disp(['Zakończono przetwarzanie mapy ', num2str(map_idx), ' z ', num2str(num_obstacles), ' przeszkodami']);
    end
    
    disp(['Zakończono przetwarzanie wszystkich konfiguracji dla mapy ', num2str(map_idx)]);
end

%% Funkcja generująca losowe przeszkody - POPRAWIONA
function obstacles = generateObstacles(binMapa, num_obstacles, block_size, start_point, end_point, width, height)
    obstacles = zeros(num_obstacles, 2);
    count = 0;
    max_attempts = 5000; % Zwiększona liczba prób
    attempts = 0;
    
    % Debug info
    disp(['Próbuję wygenerować ' num2str(num_obstacles) ' przeszkód']);
    disp(['Wymiary mapy: ' num2str(width) 'x' num2str(height)]);
    disp(['Rozmiar przeszkody: ' num2str(block_size)]);
    
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
            
            % Debug info przy każdej 5 przeszkodzie
            if mod(count, 5) == 0
                disp(['Wygenerowano ' num2str(count) ' z ' num2str(num_obstacles) ' przeszkód']);
            end
        end
        
        attempts = attempts + 1;
        
        % Debug info przy wielu próbach
        if attempts % 500 == 0
            disp(['Wykonano ' num2str(attempts) ' prób, wygenerowano ' num2str(count) ' przeszkód']);
        end
    end
    
    % Jeśli nie udało się wygenerować wszystkich przeszkód
    if count < num_obstacles
        disp(['UWAGA: Wygenerowano tylko ' num2str(count) ' z ' num2str(num_obstacles) ' przeszkód po ' num2str(attempts) ' próbach']);
    end
    
    % Przytnij tablicę do faktycznie wygenerowanych przeszkód
    obstacles = obstacles(1:count, :);
end
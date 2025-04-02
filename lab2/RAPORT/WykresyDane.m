drivingScenarioDesigner



time_values = zeros(1, 512);
roll_values = zeros(1, 512);
pitch_values = zeros(1, 512);
yaw_values = zeros(1, 512);
valid_indices = false(1, 512);


for i = 1:512
    time_values(i) = eneba(i).Time;


    for j = 1:length(eneba(i).ActorPoses)
        if eneba(i).ActorPoses(j).ActorID == 1
            roll_values(i) = eneba(i).ActorPoses(j).Roll;
            pitch_values(i) = eneba(i).ActorPoses(j).Pitch;
            yaw_values(i) = eneba(i).ActorPoses(j).Yaw;
            valid_indices(i) = true;
            break;
        end
    end
end


valid_time = time_values(valid_indices);
valid_roll = roll_values(valid_indices);
valid_pitch = pitch_values(valid_indices);
valid_yaw = yaw_values(valid_indices);


figure('Position', [100, 100, 800, 600]);


subplot(3,1,1);
plot(valid_time, valid_roll, 'r-', 'LineWidth', 2);
title('Roll dla ActorID = 1');
ylabel('Roll (stopnie)');
grid on;


subplot(3,1,2);
plot(valid_time, valid_pitch, 'g-', 'LineWidth', 2);
title('Pitch dla ActorID = 1');
ylabel('Pitch (stopnie)');
grid on;


subplot(3,1,3);
plot(valid_time, valid_yaw, 'b-', 'LineWidth', 2);
title('Yaw dla ActorID = 1');
xlabel('Czas (s)');
ylabel('Yaw (stopnie)');
grid on;


sgtitle('Orientacja dla ActorID = 1');


time_values = zeros(1, 512);
orientation_x = zeros(1, 512);
orientation_y = zeros(1, 512);
orientation_z = zeros(1, 512);
valid_indices = false(1, 512);


        for i = 1:512
            time_values(i) = eneba(i).Time;


            if ~isempty(eneba(i).INSMeasurements) && numel(eneba(i).INSMeasurements) >= 1
                try

                    orientation = eneba(i).INSMeasurements{1, 1}.Orientation;
                    orientation_x(i) = orientation(1);
                    orientation_y(i) = orientation(2);
                    orientation_z(i) = orientation(3);
                    valid_indices(i) = true;
                catch
                    % Jeśli wystąpi błąd, pomijamy ten punkt czasowy
                    continue;
                end
            end
        end

        % Filtrujemy dane tylko dla punktów czasowych z dostępnymi danymi
valid_time = time_values(valid_indices);
valid_orientation_x = orientation_x(valid_indices);
valid_orientation_y = orientation_y(valid_indices);
valid_orientation_z = orientation_z(valid_indices);

% Tworzenie figury z trzema oddzielnymi wykresami
figure('Position', [100, 100, 800, 600]); % Ustawienie rozmiaru figury

% Pierwszy subplot dla orientacji X
subplot(3,1,1);
plot(valid_time, valid_orientation_x, 'r-', 'LineWidth', 2);
title('Orientacja Roll');
ylabel('X');
grid on;

% Drugi subplot dla orientacji Y
subplot(3,1,2);
plot(valid_time, valid_orientation_y, 'g-', 'LineWidth', 2);
title('Orientacja Pitch');
ylabel('Y');
grid on;

% Trzeci subplot dla orientacji Z
subplot(3,1,3);
plot(valid_time, valid_orientation_z, 'b-', 'LineWidth', 2);
title('Orientacja Yaw');
xlabel('Czas (s)');
ylabel('Z');
grid on;

% Dodanie wspólnego tytułu dla całej figury
sgtitle('Dane orientacji z INSMeasurements');

%% load
h = load_system('simulinkGit');

% Pobranie danych czasu
t = out.tout;

% Pobranie wartości zmiennych (upewniamy się, że to wektor)
SAcc = squeeze(out.SAcc.Data);
SOr = squeeze(out.SOr.Data);
SPos = squeeze(out.SPos.Data);
SVel = squeeze(out.SVel.Data);

% Rysowanie wykresów
figure;

subplot(4,1,1);
plot(t, SAcc);
title('Acceleration (SAcc)');
xlabel('Time [s]');
ylabel('Acceleration');
grid on;

subplot(4,1,2);
plot(t, SOr);
title('Orientation (SOr)');
xlabel('Time [s]');
ylabel('Orientation');
grid on;

subplot(4,1,3);
plot(t, SPos);
title('Position (SPos)');
xlabel('Time [s]');
ylabel('Position');
grid on;

subplot(4,1,4);
plot(t, SVel);
title('Velocity (SVel)');
xlabel('Time [s]');
ylabel('Velocity');
grid on;
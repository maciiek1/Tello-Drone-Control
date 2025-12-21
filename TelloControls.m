%% --- KONFIGURACJA I INICJALIZACJA ---

% Sprawdzenie połączenia z dronem
if ~exist('drone', 'var')
    error('Brak obiektu "drone". Połącz się z dronem przed uruchomieniem skryptu.');
end

% Definicja macierzy komend (Command Matrix)
% Col 1: Typ komendy (ID)
% Col 2: Dystans [m] / Kąt [deg]
% Col 3: Prędkość [m/s]
% Col 4: Tryb WaitUntilDone (0 = false, 1 = true)
cmd = [1,  0,   0,   0;   % Takeoff
       3,  0.5, 0.5, 0;   % Move Forward 0.5m
       5,  0.5, 0.5, 0;   % Move Right 0.5m
       2,  0,   0,   0];  % Land

% Inicjalizacja zmiennych
num_commands = size(cmd, 1);
data_idx = 1;
% Prealokacja pamięci dla danych (opcjonalnie, dla optymalizacji można zwiększyć)
data = zeros(1000, 8); 

% Macierz do symulacji teoretycznej trasy [dx, dy, dz]
moves = zeros(num_commands, 3);
current_rot = 0; % Aktualny kąt obrotu (w radianach)

%% --- PĘTLA STERUJĄCA (CONTROL LOOP) ---

% Wyświetlenie instrukcji (jeśli pierwsza komenda to 0)
if cmd(1,1) == 0
    fprintf('===========================\n');
    fprintf('     DOSTĘPNE KOMENDY      \n');
    fprintf('===========================\n');
    fprintf('1: Takeoff\n');
    fprintf('2: Land\n');
    fprintf('3: Move Forward  (Dist, Speed, Wait)\n');
    fprintf('4: Move Back     (Dist, Speed, Wait)\n');
    fprintf('5: Move Right    (Dist, Speed, Wait)\n');
    fprintf('6: Move Left     (Dist, Speed, Wait)\n');
    fprintf('7: Move Up       (Dist, Speed, Wait)\n');
    fprintf('8: Move Down     (Dist, Speed, Wait)\n');
    fprintf('9: Turn          (Degrees)\n');
    fprintf('===========================\n');
else
    fprintf('Rozpoczynanie sekwencji lotu...\n');
    
    for i = 1:num_commands
        % Pobranie parametrów bieżącej komendy
        type = cmd(i, 1);
        val  = cmd(i, 2); % Dystans lub kąt
        spd  = cmd(i, 3); % Prędkość
        wait_flag = logical(cmd(i, 4)); % Konwersja 0/1 na true/false
        
        fprintf('\n--- Komenda %d/%d ---\n', i, num_commands);

        switch type
            case 1 % Takeoff
                moves(i,:) = [0, 0, 0];
                fprintf('Status: Start (Takeoff)...\n');
                takeoff(drone);
                
            case 2 % Land
                moves(i,:) = [0, 0, -1]; % Orientacyjnie w dół dla wykresu
                fprintf('Status: Lądowanie (Land)...\n');
                land(drone);
                
                % Pętla zbierania danych podczas lądowania
                t_start = tic;
                while (toc(t_start) < 5)
                    orientation = readOrientation(drone);
                    height_val = readHeight(drone);
                    speed = readSpeed(drone);
                    
                    % Zapis danych: [Roll, Pitch, Yaw, Z, Vx, Vy, Vz, Time]
                    data(data_idx, :) = [orientation, height_val, speed, toc(t_start)];
                    data_idx = data_idx + 1;
                    pause(0.05);
                end
                
            case 3 % Move Forward
                moves(i,1) = cos(current_rot) * val;
                moves(i,2) = sin(current_rot) * val;
                
                fprintf('Akcja: Przód | Dystans: %.1fm | Prędkość: %.1fm/s\n', val, spd);
                moveforward(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
                % Zbieranie danych
                t_start = tic;
                est_time = (val / spd) + 0.5;
                while (toc(t_start) < est_time)
                    orientation = readOrientation(drone);
                    height_val = readHeight(drone);
                    speed = readSpeed(drone);
                    data(data_idx, :) = [orientation, height_val, speed, toc(t_start)];
                    data_idx = data_idx + 1;
                    pause(0.05);
                end
                
            case 4 % Move Back
                moves(i,1) = -1 * cos(current_rot) * val;
                moves(i,2) = -1 * sin(current_rot) * val;
                
                fprintf('Akcja: Tył | Dystans: %.1fm\n', val);
                moveback(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
                t_start = tic;
                est_time = (val / spd) + 0.5;
                while (toc(t_start) < est_time)
                    orientation = readOrientation(drone);
                    data(data_idx, :) = [orientation, readHeight(drone), readSpeed(drone), toc(t_start)];
                    data_idx = data_idx + 1;
                    pause(0.05);
                end
                
            case 5 % Move Right
                moves(i,1) = cos(current_rot - pi/2) * val;
                moves(i,2) = sin(current_rot - pi/2) * val;
                
                fprintf('Akcja: Prawo | Dystans: %.1fm\n', val);
                moveright(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
                t_start = tic;
                est_time = (val / spd) + 0.5;
                while (toc(t_start) < est_time)
                    orientation = readOrientation(drone);
                    data(data_idx, :) = [orientation, readHeight(drone), readSpeed(drone), toc(t_start)];
                    data_idx = data_idx + 1;
                    pause(0.05);
                end
                
            case 6 % Move Left
                moves(i,1) = cos(current_rot + pi/2) * val;
                moves(i,2) = sin(current_rot + pi/2) * val;
                
                fprintf('Akcja: Lewo | Dystans: %.1fm\n', val);
                moveleft(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
                t_start = tic;
                est_time = (val / spd) + 0.5;
                while (toc(t_start) < est_time)
                    orientation = readOrientation(drone);
                    data(data_idx, :) = [orientation, readHeight(drone), readSpeed(drone), toc(t_start)];
                    data_idx = data_idx + 1;
                    pause(0.05);
                end

            case 7 % Move Up
                moves(i,3) = val;
                fprintf('Akcja: Góra | Dystans: %.1fm\n', val);
                moveup(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
            case 8 % Move Down
                moves(i,3) = -val;
                fprintf('Akcja: Dół | Dystans: %.1fm\n', val);
                movedown(drone, 'Distance', val, 'Speed', spd, 'WaitUntilDone', wait_flag);
                
            case 9 % Turn
                current_rot = current_rot + deg2rad(val);
                fprintf('Akcja: Obrót o %.1f stopni\n', val);
                turn(drone, deg2rad(val));
                
            otherwise
                fprintf('BŁĄD: Nieznana komenda (ID: %d)\n', type);
        end
    end
end

%% --- WIZUALIZACJA WYNIKÓW ---

% Przycięcie macierzy danych do faktycznej liczby próbek
valid_data = data(1:data_idx-1, :);

fprintf('\nGenerowanie wykresów...\n');

% 1. Obliczanie teoretycznej ścieżki (Cumulative Sum)
sim_positions = cumsum([0,0,0; moves]); % Dodajemy punkt startowy (0,0,0)
X_sim = sim_positions(:,1);
Y_sim = sim_positions(:,2);
Z_sim = sim_positions(:,3);

% 2. Obliczanie rzeczywistej ścieżki (Dead Reckoning)
real_pos_x = zeros(size(valid_data, 1), 1);
real_pos_y = zeros(size(valid_data, 1), 1);
real_pos_z = zeros(size(valid_data, 1), 1);

% Całkowanie numeryczne prędkości
for i = 2:size(valid_data, 1)
    dt = valid_data(i, 8) - valid_data(i-1, 8); % Delta czasu
    % Uwaga: Proste sumowanie wektorów prędkości bez uwzględnienia globalnej rotacji
    real_pos_x(i) = real_pos_x(i-1) + valid_data(i, 5) * dt;
    real_pos_y(i) = real_pos_y(i-1) + valid_data(i, 6) * dt;
    real_pos_z(i) = real_pos_z(i-1) + valid_data(i, 7) * dt;
end

% --- Rysowanie Wykresów ---

% Rysunek 1: Porównanie Trajektorii 3D
figure(1); clf;
plot3(X_sim, Y_sim, Z_sim, 'b--o', 'LineWidth', 1.5, 'DisplayName', 'Symulacja (Zadana)');
hold on;
plot3(real_pos_x, real_pos_y, real_pos_z, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Rzeczywistość (Dead Reckoning)');
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Porównanie Trajektorii Lotu: Planowana vs Rzeczywista');
legend('show');
axis equal;

% Rysunek 2: Analiza parametrów lotu
figure(2); clf;

% Orientacja
subplot(2,2,1);
plot(valid_data(:,8), valid_data(:,1:3));
title('Orientacja (Euler Angles)');
legend('Yaw', 'Pitch', 'Roll');
xlabel('Czas [s]'); ylabel('Kąt [rad]');
grid on;

% Wysokość
subplot(2,2,2);
plot(valid_data(:,8), valid_data(:,4));
title('Wysokość (Sensor)');
xlabel('Czas [s]'); ylabel('Wysokość [m]');
grid on;

% Prędkości liniowe
subplot(2,2,3);
plot(valid_data(:,8), valid_data(:,5:7));
title('Prędkości Liniowe (VX, VY, VZ)');
legend('Vx', 'Vy', 'Vz');
xlabel('Czas [s]'); ylabel('Prędkość [m/s]');
grid on;

% Pozycja obliczona
subplot(2,2,4);
plot(valid_data(:,8), real_pos_x, 'DisplayName', 'X'); hold on;
plot(valid_data(:,8), real_pos_y, 'DisplayName', 'Y');
plot(valid_data(:,8), real_pos_z, 'DisplayName', 'Z');
title('Estymowana Pozycja (X, Y, Z)');
legend('show');
xlabel('Czas [s]'); ylabel('Dystans [m]');
grid on;
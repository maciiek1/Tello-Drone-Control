filename = 'tellodron_tab.txt';

tabela = readmatrix(filename, 'DecimalSeparator', ',', 'NumHeaderLines', 2);

disp(size(tabela)); 

t = tabela(:, 1); 
x = tabela(:, 2); 
y = tabela(:, 3); 

hold off
plot(x,y)
axis square
xlim([-0.5 1.5]); 
ylim([-1 1]);
grid on;

hold on
idealx = [0 1]
idealy = [0 0]

plot(idealx, idealy)

%błąd
error = zeros(1, length(tabela));
for i = 1:length(tabela)

    if x(i) < 0
        error(i) = sqrt(x(i)^2 + y(i)^2);
    elseif x(i) > 1
        error(i) = sqrt((x(i)-1)^2 + y(i)^2);
    else
        error(i) = y(i);
    end

end

figure

plot(t,error)



% poochodna błędu i prędkości
figure

derror = diff(error);
dx = diff(x);
dy = diff(y);
dt = diff(t);

verror = derror ./dt;
vx = dx ./ dt;
vy = dy ./ dt;


t_vplot = t(1:end-1); 

figure

plot(t_vplot, derror);

figure
subplot(1,2,1)
plot(t_vplot, vx);

subplot(1,2,2)
plot(t_vplot, vy);

% Przyspieszenia
figure

dvx = diff(vx);
dvy = diff(vy);
dvt = diff(t_vplot);

ax = dvx ./ dvt;
ay = dvy ./ dvt;

t_aplot = t(1:end-2);

subplot(1,2,1)
plot(t_aplot, ax);

subplot(1,2,2)
plot(t_aplot, ay);







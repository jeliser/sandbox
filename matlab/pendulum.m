theta_0 = pi/4;
vel_0 = 0;
g = 9.80665;
l = 1.5; % 0.5m pendulum arm

y = linspace(0, 10, 500);
plot(y, (theta_0)*cos(sqrt(g/l) * y) * 180 / pi);
xlabel('Time (s)');
ylabel('Theta (deg)');
title('Pendulum');


% plot(linspace(0, 10), (pi)*cos(sqrt(9.8/0.5) * linspace(0, 10)) * 180 / pi)
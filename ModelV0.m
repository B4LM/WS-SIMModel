clc; clear; close all; 

%% Systemdaten
RPM = 2000;             %muss noch mit Tests abgeglichen werden
rm = 0.04721;           %Abstand von IMU zu Mittelpunkt in m
omega = 2*pi*RPM/60;    %Winkelgeschw.

% Fürs erste wird Fahrtgeschwindigkeit 0 angenommen, bzw konstant gehalten

%% ODE
vx_0 = 0;
vy_0 = rm*omega;
p_0   = [rm;0;0;vx_0;vy_0;omega];

interval = 0:0.1:10;

[t,p]   = ode45(@(t,p)myODE(t,p,rm,omega),interval,p_0);

%% Plotting

% for i=1:length(t)
%     a = sqrt(p(i,4)^2 * p(i,5)^2);
%     plot(t,a,'o','LineWidth',4);
%     %axis([0 11, 1500 2500]);
%     pause(0.01);
% end 

%% Funktionen

%ODE
function dp = myODE(t,p,rm,omega)
dp = zeros(size(p));
dp(1) = p(4);
dp(2) = p(5);
dp(3) = omega;
dp(4) = -cos(p(3))*rm*omega^2;
dp(5) = -sin(p(3))*rm*omega^2;
dp(6) = 0;
end



clc; clear; close all; 

%% Systemdaten
RPM = 2000;             %muss noch mit Tests abgeglichen werden
rm = 47.21;             %Abstand von IMU zu Mittelpunkt in mm
omega = 2*pi*RPM/60;    %Winkelgeschw.
rps = 2000/60;
rt = 1/rps;

% Fürs erste wird Fahrtgeschwindigkeit 0 angenommen, bzw konstant gehalten

%% ODE
vx_0 = 0;
vy_0 = rm*omega;
p_0   = [rm,0,0,vx_0,vy_0,omega];

interval = 0:(2*rt)/360:2*rt;

[t,p]   = ode45(@(t,p)myODE(t,p,rm),interval,p_0);

%% Plotting

for i=1:length(t)
    %a = sqrt(p(i,4)^2 * p(i,5)^2);
    plot(p(i,1),p(i,2),'o','LineWidth',4);
    axis([-60 60, -60 60]);
    pause(0.01);
end 
%plot(t,p(:,1),"Color","blue")
%plot(p(:,1),p(:,2),"Color","red")

%% Funktionen

%ODE
function dp = myODE(t,p,rm)
dp = zeros(size(p));
dp(1) = -rm*sin(p(3))*p(6);
dp(2) = rm*cos(p(3))*p(6);
dp(3) = p(6);
dp(4) = -cos(p(3))*rm*p(6)^2;
dp(5) = -sin(p(3))*rm*p(6)^2;
dp(6) = 0;
end



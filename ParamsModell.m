clc; clear; close all; 
%für beide Modelle

%% um in Simulink Modell zu wechseln (Modell 1, also mit Reibblöcken ist voreingestellt)
Modell=1;

%% general
g = 9.81;                    % [m/s^2]
m_ges = 1;                   % [kg]
Achsabstand = 0.15;          % [m]    
Motoruebersetzung = 100;     % [1]
B_dis = 0.15;                % [m]
Xi = 1/3;                    % [1]
I_Bot = 0.0072;              % [kg*m^2]
mue_g = 0.1;                 % [1]
Reifen_Radius = 0.04;        % [m]
I_Reifen = 1.8 * 10^-5;      % [kg*m^2]
L_B = 0.12;                  % [m]


%% Motormodell

motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4; 9.12 * 10^-8];
motor_Drehmomentkoef = 0.0035;      % [N*m / A]                     old: 0.0174;
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% Wegpunkte

StartPos = [0.5-B_dis*Xi;0.5];     
Zielpunkt = [1;1.2];

%% Startwerte
x0s = [0;
       0;
       0;
       0;
       StartPos(1);
       StartPos(2);
       0;];

x0Motor = [0;
           0];
%% Pose Ausgangsmatrix
c_pose= [0 0 0 0 1 0 0;
         0 0 0 0 0 1 0;
         0 0 0 0 0 0 1];
%-> x-pos, y-pos, theta

%% Map

myMap = binaryOccupancyMap(1.8,1.8,100);

walls = zeros(180,180);
walls(15,15:165) = 1; % Nord-Wand
walls(165,15:165) = 1; % Süd-Wand
walls(15:165,15) = 1; % West-Wand
walls(15:165,165) = 1; % Ost-Wand
walls((180-Zielpunkt(2)*100)-1:(180-Zielpunkt(2)*100)+1,(Zielpunkt(1)*100)-1:(Zielpunkt(1)*100)+1) = 1; %Zielpunkt


setOccupancy(myMap,[1 1], walls, "grid")
show(myMap)









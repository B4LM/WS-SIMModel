clc; clear; close all; 
%für DC-Motormodell und beide Robotermodelle

%% Allgemeine Parameter
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


%% Motorparameter
motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s]
motor_Drehmomentkoef = 0.0035;      % [N*m / A]              
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% Wegpunkte

StartPos = [0.5-B_dis*Xi;0.5];

%Für Positionskontrolle
Zielpunkt = [0.4;1.4];

%% Startwerte
%Rür beide Robotermodelle 
x0s = [0;
       0;
       0;
       0;
       StartPos(1);
       StartPos(2);
       0;];

%Für DC-Motor-Modell
x0Motor = [0;
           0];

%% Pose Ausgangsmatrix
c_pose= [0 0 0 0 1 0 0;
         0 0 0 0 0 1 0;
         0 0 0 0 0 0 1];
%-> x-pos, y-pos, theta

%% Occupancy Map/Arena

myMap = binaryOccupancyMap(1.8,1.8,100);

walls = zeros(180,180);
walls(15,15:165) = 1; % Nord-Wand
walls(165,15:165) = 1; % Süd-Wand
walls(15:165,15) = 1; % West-Wand
walls(15:165,165) = 1; % Ost-Wand
walls((180-Zielpunkt(2)*100)-1:(180-Zielpunkt(2)*100)+1,(Zielpunkt(1)*100)-1:(Zielpunkt(1)*100)+1) = 1; %Zielpunkt


setOccupancy(myMap,[1 1], walls, "grid")
show(myMap)


%%%%%%% safe
% 
% function [e_dis,e_angle, Distance] = fcn(thetaist,xist,xsoll)
% 
% Pos_Error = xist-xsoll;   %Distance to waypoint in global coordinates      
% angle2goal = atan2(Pos_Error(2),Pos_Error(1));
% %RotPhi = [cos(theta) +sin(theta);-sin(theta) cos(theta)];
% e_angleraw = thetaist-angle2goal; %nicht unbedingt kürzester Winkel
% 
% e_dis = norm(Pos_Error);
% e_angle = atan2(sin(e_angleraw),cos(e_angleraw));
% 
% 
% % E_bot = RotPhi * Pos_Error;
% % PID1e = E_bot(1);        %direct distance to waypoint (on body coordinates)
% % PID2e = E_bot(2);        %lateral distance to waypoint (on body coordinates)
% 
% Distance = norm(Pos_Error);
% end
% 
% function [U2, U1] = fcn(U_t, U_r)%, Achsabstand, Reifen_Radius,motor_BackEMFkoef, Motoruebersetzung
% 
% % U1 = (Motoruebersetzung / motor_BackEMFkoef) * ((ve-om*(Achsabstand/2))/Reifen_Radius);
% % U2 = (Motoruebersetzung / motor_BackEMFkoef) * ((ve+om*(Achsabstand/2))/Reifen_Radius;
% 
% % om_t = PID1e;
% % om_r = PID2e;
% 
% % U1 = (motor_BackEMFkoef/Motoruebersetzung)*(om_t-om_r);
% % U2 = (motor_BackEMFkoef/Motoruebersetzung)*(om_t+om_r);
% U1 = (U_t-U_r);
% U2 = (U_t+U_r);
% 
% % U1 = max(min(U1, 60), -60);
% % U2 = max(min(U2, 60), -60);
% end









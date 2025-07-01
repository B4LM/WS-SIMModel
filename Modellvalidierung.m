 clc; clear; close all; 

%Modellvalidierung bei eingeschwndgenem Zustand-> konstante gerade und
%kurvige Fahrt

%% Modellparameter
g = 9.81;                    % [m/s^2]
m_ges = 1;                   % [kg]
Achsabstand = 0.15;          % [m]    
Motoruebersetzung = 100;     % [1]
B_dis = 0.25;                % [m]
Xi = 2/3;                    % [1]
I_Bot = 0.0072;              % [kg*m^2]
mue_g = 0.1;                 % [1]
Reifen_Radius = 0.04;        % [m]
I_Reifen = 1.8 * 10^-5;      % [kg*m^2]
motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
motor_Drehmomentkoef = 0.0035;      % [N*m / A]                     old: 0.0174;
motor_BackEMFkoef = 0.0035;         % [V*s / rad]


%% 5V-konstante geradeaus-Fahrt
U = 5; %[V]

%mit Reibkräften
om_straight_Frb = (-Reifen_Radius*motor_Widerstand*mue_g*m_ges*Xi*g+Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef))
vel_straight_Frb = om_straight_Frb*Reifen_Radius

%ohne Reibkräften
om_straight = (Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef))
vel_straight = om_straight*Reifen_Radius

%% +/- 5V an Position drehen
U = 5; %[V]

%mit Reibkräften
om_turn_Frb = (-Reifen_Radius*motor_Widerstand*mue_g*m_ges*Xi*g+Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef))
om_bot = (2*6);

%ohne Reibkräften
om_turn = (Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef))
om_bot = (2*om_turn*Reifen_Radius)/Achsabstand
 clc; clear; close all; 

%Modellvalidierung bei eingeschwndgenem Zustand-> konstante gerade und
%kurvige Fahrt

%% Modellparameter
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
motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
motor_Drehmomentkoef = 0.0035;      % [N*m / A]                     old: 0.0174;
motor_BackEMFkoef = 0.0035;         % [V*s / rad]
L_B = 0.12; 


%% 5V-konstante geradeaus-Fahrt
U = 5; %[V]

%mit Reibkräften
om_straight_Frb = (-Reifen_Radius*motor_Widerstand*mue_g*m_ges*Xi*g+Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef));
vel_straight_Frb = om_straight_Frb*Reifen_Radius;

%ohne Reibkräften
om_straight = (Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef));
vel_straight = om_straight*Reifen_Radius;

%% +/- 5V an Position drehen
U = 5; %[V]
%%%
B1 = [(1-Xi)*B_dis; L_B/2];
B2 = [(1-Xi)*B_dis; -L_B/2];
xppos = [0.5; 0];
B1xp = xppos - B1;
B2xp = xppos - B2;
Fb1_dir = [-B1xp(2);B1xp(1)];
Fb2_dir = [-B2xp(2);B2xp(1)];
Fb1_dir_unit = Fb1_dir / norm(Fb1_dir);
Fb2_dir_unit = Fb2_dir / norm(Fb2_dir);

d1n = -(B1(1)*Fb1_dir_unit(2)-B1(2)*Fb1_dir_unit(1))/sqrt(Fb1_dir_unit(1)^2+Fb1_dir_unit(2)^2)
d2n = (B2(1)*Fb2_dir_unit(2)-B2(2)*Fb2_dir_unit(1))/sqrt(Fb2_dir_unit(1)^2+Fb2_dir_unit(2)^2)
%%%
%alpha = atan(L_B/(2*B_dis))+(pi/2);
%d1 = Xi*B_dis * sin(alpha) + (L_B/2) * cos(alpha)
%d2 = -(Xi*B_dis * sin(alpha) - (L_B/2) * cos(alpha))

% d1 = Xi*B_dis * cos(alpha) + (L_B/2) * sin(alpha)
% d2 = -(Xi*B_dis * cos(alpha) - (L_B/2) * sin(alpha))
FRB_stern = (mue_g*m_ges*g*Xi*2*(d1n+d2n)/Achsabstand);

%mit Reibkräften
om_turn_Frb = (-Reifen_Radius*motor_Widerstand*FRB_stern+Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef));
om_bot_Frb = (2*om_turn_Frb*Reifen_Radius)/Achsabstand

%ohne Reibkräften
om_turn = (Motoruebersetzung*motor_Drehmomentkoef*U)/(Motoruebersetzung^2*(motor_Widerstand*motor_Daempfung+motor_BackEMFkoef*motor_Drehmomentkoef));
om_bot = (2*om_turn*Reifen_Radius)/Achsabstand;
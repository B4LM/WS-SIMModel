clc; clear; close all; 

%% general

g = 9.81;                    % [m/s^2]
m_ges = 0.53;                % [kg]
m_last = 0.51;               % [kg]
Achsabstand = 0.15;          % [m]
Motoruebersetzung = 1/50;    % [1]


%% Motormodell

motorinertia = 0.847 * 10^-6;% [kg*m^2]
motorinductivity = 0.0005;   % [H]
motorresistance =  3;        % [Ohm]
motordamping = 0.015;        % [?]
motortorquekoef = 0.00174;   % [N*m / A]
motorBackEMFkoef = 0.00174;  % [?]

%% Reifenmodell

m_Reifen = 0.01;             % [kg]
mue = 0.002;                 % [1]
Reifen_Radius = 0.05;        % [m]

%% motor state space model

A = [0 1 0; 0 -motordamping/motorinertia motortorquekoef/motorinertia;0 -motorBackEMFkoef/motorinductivity -motorresistance/motorinductivity];
B = [0; 0; 1/motorinductivity];
CT = [0 1 0];
D = [0];

x0 = [0; 0.01; 0];

%% second try state space model

A22 = -motordamping/(motorinertia + m_Reifen * Reifen_Radius^2);
A23 = motortorquekoef/(motorinertia + m_Reifen * Reifen_Radius^2);
A32 = -motorBackEMFkoef/motorinductivity;
A33 = -motorresistance/motorinductivity;

A_neu = [0 1 0; 0 A22 A23; 0 A32 A33];
B_neu = [0; 0; 1/motorinductivity];
E_neu = [0; -((m_last + m_Reifen) * g * mue * Reifen_Radius); 0 ];
CT_nvel = [0 1 0];
CT_npos = [1 0 0];
x0_neu = [0; 0.01; 0];





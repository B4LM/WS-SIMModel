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





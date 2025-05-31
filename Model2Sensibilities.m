clc; clear; close all; 
% Modell 2 Sensibilitäten-> ohne Schleifblöcke
%% Wegpunkte

StartPos = [0.3;0.3];     
Zielpunkt = [1;1.2];

%% general

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


%% Motormodell

motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
motor_Drehmomentkoef = 0.0174;      % [N*m / A]
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% Syms

syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
syms Ue1 Ue2 real
syms p_Motoruebersetzung p_motor_traegheit p_m_ges p-Reifen_Radius p_I_Bot p_Achsabstand p_motor_Drehmomentkoef p_motor_Daempfung p_motor_Widerstand p_motor_Induktivitaet p_motor_BackEMFkoef p_I_Reifen real

x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
u_sym = [Ue1; Ue2];
params_sym = [p_Motoruebersetzung; p_motor_traegheit; p_m_ges; p-Reifen_Radius; p_I_Bot; p_Achsabstand; p_motor_Drehmomentkoef; p_motor_Daempfung; p_motor_Widerstand; p_motor_Induktivitaet; p_motor_BackEMFkoef; p_I_Reifen];

%% Startwerte
x0s = [0;
       0;
       0;
       0;
       StartPos(1);
       StartPos(2);
       0;];

%% Pose Ausgangsmatrix
c_pose= [0 0 0 0 1 0 0;
         0 0 0 0 0 1 0;
         0 0 0 0 0 0 1];
%-> x-pos, y-pos, theta

%% Diff Function

function xdot = Dynamics(x,u,Motoruebersetzung, motor_traegheit, m_ges, Reifen_Radius, I_Bot, Achsabstand, motor_Drehmomentkoef, motor_Daempfung,motor_Widerstand, motor_Induktivitaet, motor_BackEMFkoef,I_Reifen)
i_m1 = x(1);
om_m1 = x(2);
i_m2 = x(3);
om_m2 = x(4);
theta = x(7);

Ue1 = u(1);
Ue2 = u(2);

J_Mn = Motoruebersetzung^2 * motor_traegheit;
C1 = (m_ges*Reifen_Radius^2)/4 - (I_Bot*Reifen_Radius^2)/(Achsabstand^2);
C2 = (m_ges*Reifen_Radius^2)/4 + (I_Bot*Reifen_Radius^2)/(Achsabstand^2);

M11 = J_Mn + C1 + I_Reifen;
M12 = C2;
M21 = M12;
M22 = M11;

J_g = [M11 M12;
       M21 M22];

M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i_m1 - Motoruebersetzung^2 * motor_Daempfung*om_m1;
                 Motoruebersetzung * motor_Drehmomentkoef * i_m2 - Motoruebersetzung^2 * motor_Daempfung*om_m2];

om_dot = inv(J_g) * M_belastungen;

di1dt = -(motor_Widerstand/motor_Induktivitaet) * i_m1 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m1 + (1/motor_Induktivitaet)* Ue1;
dom1dt = om_dot(1);
di2dt = -(motor_Widerstand/motor_Induktivitaet) * i_m2 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m2 + (1/motor_Induktivitaet)* Ue2;
dom2dt = om_dot(2);
vx = (Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
vy = (Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
dthetadt = (Reifen_Radius/Achsabstand) * (om_m2-om_m1);

xdot = [di1dt;dom1dt;di2dt;dom2dt;vx;vy;dthetadt];
end

%% Augmented Dynamics Function

function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,12);
x_dot = Dynamics(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen);
A_jacobian = zeros(7,7);
B_jacobian = zeros(7,12);
S_dot_matrix = zeros(7,12);
for j = 1:12
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot, S_dot_matrix(:)];
end




clc; clear; close all; 

%Modellvalidierung bei eingeschwndgenem Zustand-> konstante gerade und
%kurvige Fahrt

%% Eingangsspannungen
U1 = 5; %[V]
U2 = 5; %[V]

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
motor_Drehmomentkoef = 0.0174;      % [N*m / A]
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% Modellgleichungen in eingeschwungenem Zustand

syms i1 om1 i2 om2
x = [i1 om1 i2 om2];

%Stromstärken
eq1 = motor_Widerstand * i1 + motor_BackEMFkoef*om1 == U1;
eq2 = motor_Widerstand * i2 + motor_BackEMFkoef*om2 == U2;

v_Tresh = 1e-3;
k_smooth = 5000;

eps_om = 1e-6;
eta_abs_sq = (eps_om/10)^2;
k_xp = 5 / eps_om;
delta_sig_sq = 1e-6;

OmegaSum = om1 + om2;
v_Bot = (Reifen_Radius/2)*OmegaSum;
smooth_factor = 0.5 * (1 + tanh(k_smooth * (v_Bot - v_Tresh)));
C_Frb = mue_g * m_ges * g * (Xi/2);
Frb_smooth = smooth_factor * C_Frb;

deltaOmega = om2-om1;
smooth_abs_DeltaOmega = sqrt(deltaOmega^2 + eta_abs_sq);
w = 0.5 * (1 + tanh(k_xp*(smooth_abs_DeltaOmega - eps_om)));

inv_xp_A_const = 1e-6;

inv_xp_B_turn_nun = (2/Achsabstand) * deltaOmega * OmegaSum;
inv_xp_B_turn_dun = OmegaSum^2 + delta_sig_sq;
inv_xp_B_turn = inv_xp_B_turn_nun / inv_xp_B_turn_dun;

inv_xp_smooth = (1-w) * inv_xp_A_const + w * inv_xp_B_turn;

frac_smooth = B_dis * inv_xp_smooth;

nenner_sc_alpha = sqrt(1+ frac_smooth^2);
salpha_smooth = 1 / nenner_sc_alpha;
calpha_smooth = frac_smooth / nenner_sc_alpha;

C_Tr = (2 * Reifen_Radius/Achsabstand) * (1-Xi) * B_dis;

Tr1_smooth = -Frb_smooth * (Reifen_Radius * salpha_smooth + C_Tr * calpha_smooth);
Tr2_smooth = -Frb_smooth * (Reifen_Radius * salpha_smooth - C_Tr * calpha_smooth);

M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i1 - Motoruebersetzung^2 * motor_Daempfung*om1 + Tr1_smooth;
                 Motoruebersetzung * motor_Drehmomentkoef * i2 - Motoruebersetzung^2 * motor_Daempfung*om2 + Tr2_smooth];

eq3 = M_belastungen(1) == 0;
eq4 = M_belastungen(2) == 0;

sol = solve([eq1, eq2, eq3, eq4],x);

init.i1 = 0.1; init.om1 = 20; init.i2 = 0.1; init.om2 = 20;
x_sol = [sol.i1; sol.om1; sol.i2; sol.om2];

v_Bot = (Reifen_Radius/2) * (x_sol(2) + x_sol(4));
v_Bot_num = double(subs(v_Bot));
disp(v_Bot_num)

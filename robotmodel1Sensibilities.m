clc; clear; close all; 
% Modell 2 Sensibilitäten-> ohne Schleifblöcke

%% Sim Parameters
t_span = [0 1.5];            % [s]
StartPos = [0.3;0.3];        % [m]
U_sim = [10.5; 12];          % [V]

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

%% Parameter-Struktur
p_struct.Motoruebersetzung = Motoruebersetzung;
p_struct.motor_traegheit = motor_traegheit;
p_struct.m_ges = m_ges;
p_struct.Reifen_Radius = Reifen_Radius;
p_struct.I_Bot = I_Bot;
p_struct.Achsabstand = Achsabstand;
p_struct.motor_Drehmomentkoef = motor_Drehmomentkoef;
p_struct.motor_Daempfung = motor_Daempfung;
p_struct.motor_Widerstand = motor_Widerstand;
p_struct.motor_Induktivitaet = motor_Induktivitaet;
p_struct.motor_BackEMFkoef = motor_BackEMFkoef;
p_struct.I_Reifen = I_Reifen;
p_struct.B_dis = B_dis;
p_struct.Xi = Xi; 

pnum = length(fieldnames(p_struct));

%% Symbolische Variablen
syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
syms Ue1 Ue2 real
syms p_Motoruebersetzung p_motor_traegheit p_m_ges p_Reifen_Radius p_I_Bot p_Achsabstand p_motor_Drehmomentkoef p_motor_Daempfung p_motor_Widerstand p_motor_Induktivitaet p_motor_BackEMFkoef p_I_Reifen p_B_dis p_Xi real

x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
u_sym = [Ue1; Ue2];
params_sym = [p_Motoruebersetzung; p_motor_traegheit; p_m_ges; p_Reifen_Radius; p_I_Bot; p_Achsabstand; p_motor_Drehmomentkoef; p_motor_Daempfung; p_motor_Widerstand; p_motor_Induktivitaet; p_motor_BackEMFkoef; p_I_Reifen; p_B_dis; p_Xi];
params_sym_ordered = {p_Motoruebersetzung, p_motor_traegheit, p_m_ges, p_Reifen_Radius, p_I_Bot, p_Achsabstand, p_motor_Drehmomentkoef, p_motor_Daempfung, p_motor_Widerstand, p_motor_Induktivitaet, p_motor_BackEMFkoef, p_I_Reifen, p_B_dis, p_Xi};

%% symbolische Differentialgleichung

%Träheitstensor
J_Mn_sym = p_Motoruebersetzung^2 * p_motor_traegheit;
C1_sym = (p_m_ges*p_Reifen_Radius^2)/4 - (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);
C2_sym = (p_m_ges*p_Reifen_Radius^2)/4 + (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);

M11_sym = J_Mn_sym + C1_sym + p_I_Reifen;
M12_sym = C2_sym;
M21_sym = M12_sym;
M22_sym = M11_sym;

J_g_sym = [M11_sym M12_sym;
       M21_sym M22_sym];

% Berechnung der Belastungen
M_belastungen_sym = [p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m1 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m1;
                 p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m2 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m2];

% Bestimmung von Winkelbeschleunigungen der Motoren
om_dot_sym = inv(J_g_sym) * M_belastungen_sym;

%% Berechnung der Kinematik

%Geschwindigkeit vom Achsenmitelpunkt im inertialfestem Koordinatensystem
V_M_0_sym = [(p_Reifen_Radius/2) * (om_m1 + om_m2)*cos(theta);
        (p_Reifen_Radius/2) * (om_m1 + om_m2)*sin(theta)];

%Winkelgeschwindigkeit des Roboters in der Ebene
Om_Bot_sym = (p_Reifen_Radius/p_Achsabstand) * (om_m2-om_m1);

%Rotationsmatrix vom Roboter- ins inertialfeste- Koordinatensystem
A_0B_sym = [cos(theta) -sin(theta);
            sin(theta) cos(theta)];

%Schwerpunktgeschwindigkeit aufgrund der drehung im
%Roboter-Koordinatensystem
Vs_rot_B_sym = [0; p_B_dis*p_Xi*Om_Bot_sym]; 

%Gesamt-Schwerpunktgeschwindigkeit des Roboters im inertialfesten
%Koordinatensystem
V_Bots_0_sym = V_M_0_sym + A_0B_sym*Vs_rot_B_sym;

% Aufstellen von x-dot
di1dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m1 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m1 + (1/p_motor_Induktivitaet)* Ue1;
dom1dt_sym = om_dot_sym(1);
di2dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m2 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m2 + (1/p_motor_Induktivitaet)* Ue2;
dom2dt_sym = om_dot_sym(2);
vx_sym = V_Bots_0_sym(1);
vy_sym = V_Bots_0_sym(2);
dthetadt_sym = Om_Bot_sym;

xdot_sym = [di1dt_sym;dom1dt_sym;di2dt_sym;dom2dt_sym;vx_sym;vy_sym;dthetadt_sym];

%% System- und Eingangsmatrix

A_jacobian_sym = jacobian(xdot_sym,x_sym);
B_jacobian_sym = jacobian(xdot_sym,params_sym);

A_fun = matlabFunction(A_jacobian_sym,'Vars',{x_sym,u_sym,params_sym_ordered{:}});
B_fun = matlabFunction(B_jacobian_sym,'Vars',{x_sym,u_sym,params_sym_ordered{:}});

%% Startwerte
x0 = [0;
       0;
       0;
       0;
       StartPos(1);
       StartPos(2);
       0;];

S0 = zeros(7,pnum);
X_aug_0 = [x0;S0(:)];

%% ODE-Sim
options = odeset('RelTol',1e-6,'AbsTol',1e-8); %ziemlich genaue Toleranzen, sollte auch weniger möglich sein
[T,X_aug_sol] = ode45(@(t, X_aug) AugmentedDynamics(t, X_aug, U_sim, p_struct, A_fun, B_fun,pnum), t_span, X_aug_0, options);

x_traj = X_aug_sol(:,1:7);
S_sol = X_aug_sol(:,8:end);

num_t_steps = length(T);
S_traj_tensor = zeros(num_t_steps,7,pnum);
for i = 1:num_t_steps
    S_traj_tensor(i,:,:) = reshape(S_sol(i,:), 7, pnum);
end

%% Ergebnisse - Parametersensitiviäten der Zustände über eine Trajektorie
p_list = fieldnames(p_struct);
p_nom_values = zeros(length(p_list),1);
for k = 1:length(p_nom_values)
    p_nom_values(k) = p_struct.(p_list{k});
end

s_list = {'i_m1','om_m1','i_m2','om_m2','xpos','ypos','theta'};
s_list_latex = {'$i_{M1}$','$\omega_{M1}$','$i_{M2}$','$\omega_{M2}$','$x_{pos}$','$y_{pos}$','$\theta$'};
p_list_latex = {'Motoruebersetzung', '$Traegheit_M$', '$m_{ges}$', '$R_{Reifen}$', '$I_{Bot}$', 'Achsabstand', '$Drehmomentkoef_M$', '$Daempfung_M$', '$\Omega_M$', '$Induktivitaet_M$', '$BackEMFkoef_M$', '$I_{Reifen}$','$B_{dis}$','$\xi$'};

S_rel_tensor = zeros(size(S_traj_tensor));
for kp = 1:length(p_list)
    S_rel_tensor(:,:,kp) = S_traj_tensor(:,:,kp) * p_nom_values(kp);
end


%% Plotten der Ergebnisse mit ineraktiver Abfrage nach welchem Zustand

again = true;
counter = 0;
h_plot = gobjects(1, length(p_list));  % Preallocate Grafik-Handles
colors = [
    0.0000, 0.4470, 0.7410;  % Blau
    0.8500, 0.3250, 0.0980;  % Orange
    0.9290, 0.6940, 0.1250;  % Gelb
    0.4940, 0.1840, 0.5560;  % Violett
    0.4660, 0.6740, 0.1880;  % Grün
    0.3010, 0.7450, 0.9330;  % Hellblau
    0.6350, 0.0780, 0.1840;  % Rot
    0.0000, 0.0000, 0.0000;  % Schwarz
    0.9060, 0.1610, 0.5410;  % Pink
    0.6000, 0.6000, 0.6000;  % Grau
    0.0000, 0.6000, 0.0000;  % Dunkelgrün
    0.0000, 0.0000, 0.5000;  % Dunkelblau
    1.0000, 0.8430, 0.0000;  % Gold
    0.5000, 0.0000, 0.0000;  % Dunkelrot
];

while again == true
    while true
        if counter ~=0
            stop_input = input([newline 'another plot? (Y / N)'], 's');
            if strcmpi(stop_input, "Y")
                close all;
                break;
            elseif strcmpi(stop_input, "N")
                again = false;
                break;
            else
                disp('input not valid');
            end
        else
            break
        end
    end

    if ~again
        break;
    end

    while true
        state_input = input([newline 'chose interested state:' newline 'i_m1-> 1' newline 'om_m1-> 2' newline 'i_m2-> 3' newline 'om_m2-> 4' newline 'xpos-> 5' newline 'ypos-> 6' newline 'theta-> 7' newline 'stop plotting-> End' newline 'input:'], 's');
        if strcmpi(state_input, "End")
            again = false;
            break;
        end
        state_val = str2double(state_input);
        if ~isnan(state_val) && mod(state_val, 1) == 0 && state_val >= 1 && state_val <= 7
            state_idx = state_val;
            break;
        else
            disp('input not valid');
        end
    end

    if ~again
        break;
    end

    counter = counter+1;
    figure;
    hold on
    for p = 1:length(p_list)
        h_plot(p) = plot(T, S_rel_tensor(:, state_idx, p), 'Color', colors(p,:), 'DisplayName', p_list_latex{p});
    end
    title(['Relative Parameter-Sensitivitaet von ',s_list_latex{state_idx}], 'Interpreter', 'latex');
    xlabel('time in [s]');
    ylabel('$p_k * S(p_k,t)$', 'Interpreter', 'latex')
    legend(h_plot, 'Interpreter', 'latex', 'Location', 'eastoutside');
    grid on;
end




%% Dynamik- Differentailgleichung, wird verwendet für erweiterte Differentialgleichng-> Simulation

function xdot = Dynamics(x,u,Motoruebersetzung, motor_traegheit, m_ges, Reifen_Radius, I_Bot, Achsabstand, motor_Drehmomentkoef, motor_Daempfung,motor_Widerstand, motor_Induktivitaet, motor_BackEMFkoef,I_Reifen,B_dis,Xi)

% Input
%x-vec:
i_m1 = x(1);
om_m1 = x(2);
i_m2 = x(3);
om_m2 = x(4);
theta = x(7);

%Spannungs-Eingang
Ue1 = u(1);
Ue2 = u(2);

%Träheitstensor
J_Mn = Motoruebersetzung^2 * motor_traegheit;
C1 = (m_ges*Reifen_Radius^2)/4 - (I_Bot*Reifen_Radius^2)/(Achsabstand^2);
C2 = (m_ges*Reifen_Radius^2)/4 + (I_Bot*Reifen_Radius^2)/(Achsabstand^2);

M11 = J_Mn + C1 + I_Reifen;
M12 = C2;
M21 = M12;
M22 = M11;

J_g = [M11 M12;
       M21 M22];

% Berechnung der Belastungen
M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i_m1 - Motoruebersetzung^2 * motor_Daempfung*om_m1;
                 Motoruebersetzung * motor_Drehmomentkoef * i_m2 - Motoruebersetzung^2 * motor_Daempfung*om_m2];

% Bestimmung von Winkelbeschleunigungen der Motoren
om_dot = inv(J_g) * M_belastungen;

% Berechnung der Kinematik
%Geschwindigkeit vom Achsenmitelpunkt im inertialfestem Koordinatensystem
V_M_0 = [(Reifen_Radius/2) * (om_m1 + om_m2)*cos(theta);
        (Reifen_Radius/2) * (om_m1 + om_m2)*sin(theta)];

%Winkelgeschwindigkeit des Roboters in der Ebene
Om_Bot = (Reifen_Radius/Achsabstand) * (om_m2-om_m1);

%Rotationsmatrix vom Roboter- ins inertialfeste- Koordinatensystem
A_0B = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];

%Schwerpunktgeschwindigkeit aufgrund der drehung im
%Roboter-Koordinatensystem
Vs_rot_B = [0; B_dis*Xi*Om_Bot]; 

%Gesamt-Schwerpunktgeschwindigkeit des Roboters im inertialfesten
%Koordinatensystem
V_Bots_0 = V_M_0 + A_0B*Vs_rot_B;

% Aufstellen von x-dot
di1dt = -(motor_Widerstand/motor_Induktivitaet) * i_m1 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m1 + (1/motor_Induktivitaet)* Ue1;
dom1dt = om_dot(1);
di2dt = -(motor_Widerstand/motor_Induktivitaet) * i_m2 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m2 + (1/motor_Induktivitaet)* Ue2;
dom2dt = om_dot(2);
vx = V_Bots_0(1);
vy = V_Bots_0(2);
dthetadt = Om_Bot;

xdot = [di1dt;dom1dt;di2dt;dom2dt;vx;vy;dthetadt];
end

%% Erweiterte Dynamik Funktion
%Diese Funktion dient als Differentialgleichung für den ODE-Solver, da die Sensitivitätsmatrix die ursprünglichen Zustände benötigt, wird
%in X_aug beide Zustandsräume gespeichert
function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct, A_fun, B_fun,pnum)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,pnum);

x_dot = Dynamics(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen,p_struct.B_dis,p_struct.Xi);
A_jacobian = A_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen,p_struct.B_dis,p_struct.Xi);
B_jacobian = B_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen,p_struct.B_dis,p_struct.Xi);

S_dot_matrix = zeros(7,pnum);
for j = 1:pnum
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot; S_dot_matrix(:)];
end




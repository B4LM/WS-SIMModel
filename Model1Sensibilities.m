clc; clear; close all; 
% Modell 1 Sensibilitäten-> mit Schleifblöcke!

%% Sim Parameters
t_span = [0 1.5];            % [s]
StartPos = [0.3;0.3];        % [m]
U_sim = [10.5; 12];          % [V]

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


%% Params_struct
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
p_struct.mue_g = mue_g;
p_struct.Xi = Xi;
p_struct.B_dis = B_dis;
p_struct.g = g;
p_struct.L_B = L_B;

%% Syms

syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
syms Ue1 Ue2 real
syms p_Motoruebersetzung p_motor_traegheit p_m_ges p_Reifen_Radius p_I_Bot p_Achsabstand p_motor_Drehmomentkoef p_motor_Daempfung p_motor_Widerstand p_motor_Induktivitaet p_motor_BackEMFkoef p_I_Reifen p_mue_g p_Xi p_B_dis p_g p_L_B real %p_L_B

x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
u_sym = [Ue1; Ue2];
params_sym = [p_Motoruebersetzung; p_motor_traegheit; p_m_ges; p_Reifen_Radius; p_I_Bot; p_Achsabstand; p_motor_Drehmomentkoef; p_motor_Daempfung; p_motor_Widerstand; p_motor_Induktivitaet; p_motor_BackEMFkoef; p_I_Reifen; p_mue_g; p_Xi; p_B_dis; p_g; p_L_B];%; p_L_B
params_sym_ordered = {p_Motoruebersetzung, p_motor_traegheit, p_m_ges, p_Reifen_Radius, p_I_Bot, p_Achsabstand, p_motor_Drehmomentkoef, p_motor_Daempfung, p_motor_Widerstand, p_motor_Induktivitaet, p_motor_BackEMFkoef, p_I_Reifen,p_mue_g, p_Xi, p_B_dis, p_g, p_L_B};%, p_L_B

pnum = length(params_sym);
%% xdot-symbolic
% J_Mn_sym = p_Motoruebersetzung^2 * p_motor_traegheit;
% C1_sym = (p_m_ges*p_Reifen_Radius^2)/4 - (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);
% C2_sym = (p_m_ges*p_Reifen_Radius^2)/4 + (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);
% 
% M11_sym = J_Mn_sym + C1_sym + p_I_Reifen;
% M12_sym = C2_sym;
% M21_sym = M12_sym;
% M22_sym = M11_sym;
% 
% J_g_sym = [M11_sym M12_sym;
%            M21_sym M22_sym];
% 
% v_Tresh_sym = 1e-3;
% k_smooth_sym = 5000;
% 
% eps_om_sym = 1e-6;
% eta_abs_sq_sym = (eps_om_sym/10)^2;
% k_xp_sym = 5 / eps_om_sym;
% delta_sig_sq_sym = 1e-6;
% 
% OmegaSum_sym = om_m1 + om_m2;
% v_Bot_sym = (p_Reifen_Radius/2)*OmegaSum_sym;
% smooth_factor_sym = 0.5 * (1 + tanh(k_smooth_sym * (v_Bot_sym - v_Tresh_sym)));
% C_Frb_sym = p_mue_g * p_m_ges * p_g * (p_Xi/2);
% Frb_smooth_sym = smooth_factor_sym * C_Frb_sym;
% 
% deltaOmega_sym = om_m2-om_m1;
% smooth_abs_DeltaOmega_sym = sqrt(deltaOmega_sym^2 + eta_abs_sq_sym);
% w_sym = 0.5 * (1 + tanh(k_xp_sym*(smooth_abs_DeltaOmega_sym - eps_om_sym)));
% 
% inv_xp_A_const_sym = 1e-6;
% 
% inv_xp_B_turn_nun_sym = (2/p_Achsabstand) * deltaOmega_sym * OmegaSum_sym;
% inv_xp_B_turn_dun_sym = OmegaSum_sym^2 + delta_sig_sq_sym;
% inv_xp_B_turn_sym = inv_xp_B_turn_nun_sym / inv_xp_B_turn_dun_sym;
% 
% inv_xp_smooth_sym = (1-w_sym) * inv_xp_A_const_sym + w_sym * inv_xp_B_turn_sym;
% 
% frac_smooth_sym = p_B_dis * inv_xp_smooth_sym;
% 
% nenner_sc_alpha_sym = sqrt(1+ frac_smooth_sym^2);
% salpha_smooth_sym = 1 / nenner_sc_alpha_sym;
% calpha_smooth_sym = frac_smooth_sym / nenner_sc_alpha_sym;
% 
% C_Tr_sym = (2 * p_Reifen_Radius/p_Achsabstand) * (1-p_Xi) * p_B_dis;
% 
% Tr1_smooth_sym = -Frb_smooth_sym * (p_Reifen_Radius * salpha_smooth_sym + C_Tr_sym * calpha_smooth_sym);
% Tr2_smooth_sym = -Frb_smooth_sym * (p_Reifen_Radius * salpha_smooth_sym - C_Tr_sym * calpha_smooth_sym);
% 
% M_belastungen_sym = [p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m1 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m1+ Tr1_smooth_sym;
%                      p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m2 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m2+ Tr2_smooth_sym];
% 
% om_dot_sym = inv(J_g_sym) * M_belastungen_sym;
% 
% di1dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m1 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m1 + (1/p_motor_Induktivitaet)* Ue1;
% dom1dt_sym = om_dot_sym(1);
% di2dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m2 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m2 + (1/p_motor_Induktivitaet)* Ue2;
% dom2dt_sym = om_dot_sym(2);
% vx_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
% vy_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
% dthetadt_sym = (p_Reifen_Radius/p_Achsabstand) * (om_m2-om_m1);
% 
% xdot_sym = [di1dt_sym;dom1dt_sym;di2dt_sym;dom2dt_sym;vx_sym;vy_sym;dthetadt_sym];
%% huhu
J_Mn_sym = p_Motoruebersetzung^2 * p_motor_traegheit;
C1_sym = (p_m_ges*p_Reifen_Radius^2)/4 - (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);
C2_sym = (p_m_ges*p_Reifen_Radius^2)/4 + (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);

M11_sym = J_Mn_sym + C1_sym + p_I_Reifen;
M12_sym = C2_sym;
M21_sym = M12_sym;
M22_sym = M11_sym;

J_g_sym = [M11_sym M12_sym;
           M21_sym M22_sym];

% Grenzgeschwindigkeit gegen 0
v_Tresh_sym = 1e-3;
% Grenzwinkelgeschwindigkeit gegen 0
omega_Tresh_sym = 1e-3;
% Grenzwinkelgeschwindigkeits-Unterschied
eps_om_sym = 1e-6;
% Steigungen für Glättungsfunktionen
k_smooth_sym = 500; %5000
k_xppos_sym = 100;
k_xp_sym = 500; %5 / eps_om_sym
% konstante Nenner-Erwiterungen für Singlaritäten
delta_sig_sq_sym = 1e-6;
xp_delta_sym = 1e-6;
eta_abs_sq_sym = (eps_om_sym/10)^2;

% Angriffspunkte der Reibngskräfte
B1_sym = [(1-p_Xi)*p_B_dis; p_L_B/2];
B2_sym = [(1-p_Xi)*p_B_dis; -p_L_B/2];

% Berechnung der reibungskraft-Angriffswinkel über Ermittlung von
% Geschwindigkeitspol xp
OmegaSum_sym = om_m1 + om_m2;
deltaOmega_sym = om_m2-om_m1;
smooth_abs_DeltaOmega_sym = sqrt(deltaOmega_sym^2 + eta_abs_sq_sym);

v_Bot_sym = (p_Reifen_Radius/2)*OmegaSum_sym;
omega_Bot_sym = (p_Reifen_Radius/p_Achsabstand)*smooth_abs_DeltaOmega_sym;

% Glättungsfunktionen für Reibungskraft-> keine Reibung bei v_Bot / omega_Bot =0
smooth_factor_straight_sym = 0.5 * (1 + tanh(k_smooth_sym * (v_Bot_sym - v_Tresh_sym)));
smooth_factor_spin_sym = 0.5 * (1 + tanh(k_smooth_sym * (omega_Bot_sym - omega_Tresh_sym)));
C_Frb_sym = p_mue_g * p_m_ges * p_g * (p_Xi/2);
Frb_smooth_straight_sym  = smooth_factor_straight_sym  * C_Frb_sym ;
Frb_smooth_spin_sym  = smooth_factor_spin_sym  * C_Frb_sym ;

% Bestimmung von Geschwndigkeitspol xp
w_sym  = 0.5 * (1 + tanh(k_xp_sym *(smooth_abs_DeltaOmega_sym  - eps_om_sym )));

inv_xp_A_const_sym  = 1e-6;

inv_xp_B_turn_nun_sym  = (2/p_Achsabstand) * deltaOmega_sym * OmegaSum_sym ;
inv_xp_B_turn_dun_sym  = OmegaSum_sym ^2 + delta_sig_sq_sym ;
inv_xp_B_turn_sym  = inv_xp_B_turn_nun_sym  / inv_xp_B_turn_dun_sym ;

inv_xp_smooth_sym  = (1-w_sym) * inv_xp_A_const_sym + w_sym * inv_xp_B_turn_sym;
xp_smooth_sym = 1./(inv_xp_smooth_sym + xp_delta_sym);

xppos_translation_sym = [-p_Xi*p_B_dis; xp_smooth_sym];
xppos_turnonpint_sym = [-p_Xi*p_B_dis; 0];

s_sym = 1- tanh((k_xppos_sym * OmegaSum_sym)^2);
xppos_sym = (1-s_sym)* xppos_translation_sym + s_sym * xppos_turnonpint_sym;

% Bestimmung von angriffswinkel von Reibkräften
B1xp_sym = xppos_sym - B1_sym;
B2xp_sym = xppos_sym - B2_sym;
Fb1_dir_sym = [B1xp_sym(2);-B1xp_sym(1)];
Fb2_dir_sym = [B2xp_sym(2);-B2xp_sym(1)];
Fb1_dir_unit_sym = Fb1_dir_sym / norm(Fb1_dir_sym);
Fb2_dir_unit_sym = Fb2_dir_sym / norm(Fb2_dir_sym);
FRB1spin_sym = Frb_smooth_spin_sym * Fb1_dir_unit_sym;
FRB2spin_sym = Frb_smooth_spin_sym * Fb2_dir_unit_sym;

% Durch reibungskräfte resultierende Momente am Roboter
M_FR1v_sym = cross([B1_sym;0],[FRB1spin_sym;0]);
M_FR1_sym = M_FR1v_sym(3);
M_FR2v_sym = cross([B2_sym;0],[FRB2spin_sym;0]);
M_FR2_sym = M_FR2v_sym(3);

% Gesamte Momenten-Belastung aufgrund der reibung bei Kontaktpunkten
Tr1_smooth_sym = -(Frb_smooth_straight_sym* p_Reifen_Radius/2) * (Fb1_dir_unit_sym(1) + Fb2_dir_unit_sym(1)) + w_sym*(p_Reifen_Radius/p_Achsabstand)*(M_FR1_sym+M_FR2_sym);
Tr2_smooth_sym = -(Frb_smooth_straight_sym* p_Reifen_Radius/2) * (Fb1_dir_unit_sym(1) + Fb2_dir_unit_sym(1)) - w_sym*(p_Reifen_Radius/p_Achsabstand)*(M_FR1_sym+M_FR2_sym);

% Gesamte Belastungen an den Motoren
M_belastungen_sym = [p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m1 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m1 + Tr1_smooth_sym;
                     p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m2 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m2 + Tr2_smooth_sym];

% Bestimmung von Winkelbeschleunigungen der Motoren
om_dot_sym = J_g_sym \ M_belastungen_sym;

di1dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m1 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m1 + (1/p_motor_Induktivitaet)* Ue1;
dom1dt_sym = om_dot_sym(1);
di2dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m2 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m2 + (1/p_motor_Induktivitaet)* Ue2;
dom2dt_sym = om_dot_sym(2);
vx_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
vy_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
dthetadt_sym = (p_Reifen_Radius/p_Achsabstand) * (om_m2-om_m1);

xdot_sym = [di1dt_sym;dom1dt_sym;di2dt_sym;dom2dt_sym;vx_sym;vy_sym;dthetadt_sym];
disp("symdone!")

%% A and B matrix

A_jacobian_sym = jacobian(xdot_sym,x_sym);
disp('A_jacobian_sym done')
J_pattern_sym = (A_jacobian_sym ~= 0);
J_pattern = double(J_pattern_sym);
B_jacobian_sym = jacobian(xdot_sym,params_sym);
disp('B_jacobian_sym done')
A_fun = matlabFunction(A_jacobian_sym,'Vars',{x_sym,u_sym,params_sym_ordered{:}});
disp('A_fun done')
B_fun = matlabFunction(B_jacobian_sym,'Vars',{x_sym,u_sym,params_sym_ordered{:}});
disp('B_fun done')

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
%% Codegen init
disp('Starte Codegen-Prozess...');

ARGS = { ...
    coder.typeof(zeros(7,1)), ... % x (Zustandsvektor)
    coder.typeof(zeros(2,1)), ... % u (Eingangsvektor)
    coder.typeof(1.0), ...        % Motoruebersetzung (Skalar)
    coder.typeof(1.0), ...        % motor_traegheit
    coder.typeof(1.0), ...        % m_ges
    coder.typeof(1.0), ...        % Reifen_Radius
    coder.typeof(1.0), ...        % I_Bot
    coder.typeof(1.0), ...        % Achsabstand
    coder.typeof(1.0), ...        % motor_Drehmomentkoef
    coder.typeof(1.0), ...        % motor_Daempfung
    coder.typeof(1.0), ...        % motor_Widerstand
    coder.typeof(1.0), ...        % motor_Induktivitaet
    coder.typeof(1.0), ...        % motor_BackEMFkoef
    coder.typeof(1.0), ...        % mue_g
    coder.typeof(1.0), ...        % g
    coder.typeof(1.0), ...        % Xi
    coder.typeof(1.0), ...        % B_dis
    coder.typeof(1.0), ...        % I_Reifen
    coder.typeof(1.0)  ...        % L_B
};

codegen PSA_dynamics_codegen -args ARGS -report

disp('Codegen abgeschlossen. Starte ODE-Simulation...');

%% ODE-Sim
A_handle = @(t, x) A_fun(x, U_sim, p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen,p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g, p_struct.L_B);
options = odeset('RelTol',1e-3, 'Jacobian', A_handle, 'JPattern', J_pattern);%,'AbsTol',auto
[T,X_aug_sol] = ode15s(@(t, X_aug) AugmentedDynamics(t, X_aug, U_sim, p_struct, A_fun, B_fun, pnum), t_span, X_aug_0, options);

x_traj = X_aug_sol(:,1:7);
S_sol = X_aug_sol(:,8:end);

num_t_steps = length(T);
S_traj_tensor = zeros(num_t_steps,7,pnum);
for i = 1:num_t_steps
    S_traj_tensor(i,:,:) = reshape(S_sol(i,:), 7, pnum);
end

%% plot-solutions - relative Sensitivities
p_list = fieldnames(p_struct);
p_nom_values = zeros(length(p_list),1);
for k = 1:length(p_nom_values)
    p_nom_values(k) = p_struct.(p_list{k});
end
s_list = {'i_m1','om_m1','i_m2','om_m2','xpos','ypos','theta'};
s_list_latex = {'$i_{M1}$','$\omega_{M1}$','$i_{M2}$','$\omega_{M2}$','$x_{pos}$','$y_{pos}$','$\theta$'};
p_list_latex = {'Motoruebersetzung', '$Traegheit_M$', '$m_{ges}$', '$R_{Reifen}$', '$I_{Bot}$', 'Achsabstand', '$Drehmomentkoef_M$', '$Daempfung_M$', '$\Omega_M$', '$Induktivitaet_M$', '$BackEMFkoef_M$', '$I_{Reifen}$', '$\mu_g$', '$Xi$', '$B_{dis}$', 'g', '$L_B$'};%, '$L_B$'

S_rel_tensor = zeros(size(S_traj_tensor));
for kp = 1:length(p_list)
    S_rel_tensor(:,:,kp) = S_traj_tensor(:,:,kp) * p_nom_values(kp);
end

% plot the sensibilities of a state to every parameter
again = true;
counter = 0;
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
        plot(T,S_rel_tensor(:,state_idx,p))
    end
    title(['relative parameter-sensitvity of ',s_list_latex{state_idx}], 'Interpreter', 'latex');
    xlabel('time in [s]');
    ylabel('$p_k * \frac{dS}{dp_k}$', 'Interpreter', 'latex')
    legend(p_list_latex,'Interpreter', 'latex', 'Location', 'eastoutside');
    grid on;
end



%% Diff Function

% function xdot = Dynamics(x,u,Motoruebersetzung, motor_traegheit, m_ges, Reifen_Radius, I_Bot, Achsabstand, motor_Drehmomentkoef, motor_Daempfung,motor_Widerstand, motor_Induktivitaet, motor_BackEMFkoef,I_Reifen,mue_g,  Xi, B_dis,g)
% i_m1 = x(1);
% om_m1 = x(2);
% i_m2 = x(3);
% om_m2 = x(4);
% theta = x(7);
% 
% Ue1 = u(1);
% Ue2 = u(2);
% 
% J_Mn = Motoruebersetzung^2 * motor_traegheit;
% C1 = (m_ges*Reifen_Radius^2)/4 - (I_Bot*Reifen_Radius^2)/(Achsabstand^2);
% C2 = (m_ges*Reifen_Radius^2)/4 + (I_Bot*Reifen_Radius^2)/(Achsabstand^2);
% 
% M11 = J_Mn + C1 + I_Reifen;
% M12 = C2;
% M21 = M12;
% M22 = M11;
% 
% J_g = [M11 M12;
%        M21 M22];
% 
% v_Tresh = 1e-3;
% k_smooth = 5000;
% 
% eps_om = 1e-6;
% eta_abs_sq = (eps_om/10)^2;
% k_xp = 5 / eps_om;
% delta_sig_sq = 1e-6;
% 
% OmegaSum = om_m1 + om_m2;
% v_Bot = (Reifen_Radius/2)*OmegaSum;
% smooth_factor = 0.5 * (1 + tanh(k_smooth * (v_Bot - v_Tresh)));
% C_Frb = mue_g * m_ges * g * (Xi/2);
% Frb_smooth = smooth_factor * C_Frb;
% 
% deltaOmega = om_m2-om_m1;
% smooth_abs_DeltaOmega = sqrt(deltaOmega^2 + eta_abs_sq);
% w = 0.5 * (1 + tanh(k_xp*(smooth_abs_DeltaOmega - eps_om)));
% 
% inv_xp_A_const = 1e-6;
% 
% inv_xp_B_turn_nun = (2/Achsabstand) * deltaOmega * OmegaSum;
% inv_xp_B_turn_dun = OmegaSum^2 + delta_sig_sq;
% inv_xp_B_turn = inv_xp_B_turn_nun / inv_xp_B_turn_dun;
% 
% inv_xp_smooth = (1-w) * inv_xp_A_const + w * inv_xp_B_turn;
% 
% frac_smooth = B_dis * inv_xp_smooth;
% 
% nenner_sc_alpha = sqrt(1+ frac_smooth^2);
% salpha_smooth = 1 / nenner_sc_alpha;
% calpha_smooth = frac_smooth / nenner_sc_alpha;
% 
% C_Tr = (2 * Reifen_Radius/Achsabstand) * (1-Xi) * B_dis;
% 
% Tr1_smooth = -Frb_smooth * (Reifen_Radius * salpha_smooth + C_Tr * calpha_smooth);
% Tr2_smooth = -Frb_smooth * (Reifen_Radius * salpha_smooth - C_Tr * calpha_smooth);
% 
% M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i_m1 - Motoruebersetzung^2 * motor_Daempfung*om_m1 + Tr1_smooth;
%                  Motoruebersetzung * motor_Drehmomentkoef * i_m2 - Motoruebersetzung^2 * motor_Daempfung*om_m2 + Tr2_smooth];
% 
% om_dot = J_g \ M_belastungen;
% 
% di1dt = -(motor_Widerstand/motor_Induktivitaet) * i_m1 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m1 + (1/motor_Induktivitaet)* Ue1;
% dom1dt = om_dot(1);
% di2dt = -(motor_Widerstand/motor_Induktivitaet) * i_m2 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m2 + (1/motor_Induktivitaet)* Ue2;
% dom2dt = om_dot(2);
% vx = (Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
% vy = (Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
% dthetadt = (Reifen_Radius/Achsabstand) * (om_m2-om_m1);
% 
% xdot = [di1dt;dom1dt;di2dt;dom2dt;vx;vy;dthetadt];
% end

%% hihi

function xdot = Dynamics(x,u,Motoruebersetzung, motor_traegheit, m_ges, Reifen_Radius, I_Bot, Achsabstand, motor_Drehmomentkoef, motor_Daempfung,motor_Widerstand, motor_Induktivitaet, motor_BackEMFkoef,mue_g, g, Xi, B_dis,I_Reifen,L_B)

%x-vec:
i_m1 = x(1);
om_m1 = x(2);
i_m2 = x(3);
om_m2 = x(4);
theta = x(7);

%Spannungs-Eingang
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

% Grenzgeschwindigkeit gegen 0
v_Tresh = 1e-3;
% Grenzwinkelgeschwindigkeit gegen 0
omega_Tresh = 1e-3;
% Grenzwinkelgeschwindigkeits-Unterschied
eps_om = 1e-6;
% Steigungen für Glättungsfunktionen
k_smooth = 500; %5000
k_xppos = 100;
k_xp = 500;% 5 / eps_om_sym
% konstante Nenner-Erwiterungen für Singlaritäten
delta_sig_sq = 1e-6;
xp_delta = 1e-6;
eta_abs_sq = (eps_om/10)^2;

% Angriffspunkte der Reibngskräfte
B1 = [(1-Xi)*B_dis; L_B/2];
B2 = [(1-Xi)*B_dis; -L_B/2];

% Berechnung der reibungskraft-Angriffswinkel über Ermittlung von
% Geschwindigkeitspol xp
OmegaSum = om_m1 + om_m2;
deltaOmega = om_m2-om_m1;
smooth_abs_DeltaOmega = sqrt(deltaOmega^2 + eta_abs_sq);

v_Bot = (Reifen_Radius/2)*OmegaSum;
omega_Bot = (Reifen_Radius/Achsabstand)*smooth_abs_DeltaOmega;

% Glättungsfunktionen für Reibungskraft-> keine Reibung bei v_Bot / omega_Bot =0
smooth_factor_straight = 0.5 * (1 + tanh(k_smooth * (v_Bot - v_Tresh)));
smooth_factor_spin = 0.5 * (1 + tanh(k_smooth * (omega_Bot - omega_Tresh)));
C_Frb = mue_g * m_ges * g * (Xi/2);
Frb_smooth_straight = smooth_factor_straight * C_Frb;
Frb_smooth_spin = smooth_factor_spin * C_Frb;

% Bestimmung von Geschwndigkeitspol xp
w = 0.5 * (1 + tanh(k_xp*(smooth_abs_DeltaOmega - eps_om)));

inv_xp_A_const = 1e-6;

inv_xp_B_turn_nun = (2/Achsabstand) * deltaOmega * OmegaSum;
inv_xp_B_turn_dun = OmegaSum^2 + delta_sig_sq;
inv_xp_B_turn = inv_xp_B_turn_nun / inv_xp_B_turn_dun;

inv_xp_smooth = (1-w) * inv_xp_A_const + w * inv_xp_B_turn;
xp_smooth = 1./(inv_xp_smooth+xp_delta);

xppos_translation = [-Xi*B_dis; xp_smooth];
xppos_turnonpint = [-Xi*B_dis; 0];

s = 1- tanh((k_xppos * OmegaSum)^2);
xppos = (1-s)* xppos_translation + s * xppos_turnonpint;

% Bestimmung von angriffswinkel von Reibkräften
B1xp = xppos - B1;
B2xp = xppos - B2;
Fb1_dir = [B1xp(2);-B1xp(1)];
Fb2_dir = [B2xp(2);-B2xp(1)];
Fb1_dir_unit = Fb1_dir / norm(Fb1_dir);
Fb2_dir_unit = Fb2_dir / norm(Fb2_dir);
FRB1spin = Frb_smooth_spin * Fb1_dir_unit;
FRB2spin = Frb_smooth_spin * Fb2_dir_unit;

% Durch reibungskräfte resultierende Momente am Roboter
M_FR1v = cross([B1;0],[FRB1spin;0]);
M_FR1 = M_FR1v(3);
M_FR2v = cross([B2;0],[FRB2spin;0]);
M_FR2 = M_FR2v(3);

% Gesamte Momenten-Belastung aufgrund der reibung bei Kontaktpunkten
Tr1_smooth = -(Frb_smooth_straight* Reifen_Radius/2) * (Fb1_dir_unit(1) + Fb2_dir_unit(1)) + w*(Reifen_Radius/Achsabstand)*(M_FR1+M_FR2);
Tr2_smooth = -(Frb_smooth_straight* Reifen_Radius/2) * (Fb1_dir_unit(1) + Fb2_dir_unit(1)) - w*(Reifen_Radius/Achsabstand)*(M_FR1+M_FR2);

% Gesamte Belastungen an den Motoren
M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i_m1 - Motoruebersetzung^2 * motor_Daempfung*om_m1 + Tr1_smooth;
                 Motoruebersetzung * motor_Drehmomentkoef * i_m2 - Motoruebersetzung^2 * motor_Daempfung*om_m2 + Tr2_smooth];

% Bestimmung von Winkelbeschleunigungen der Motoren
om_dot = J_g \ M_belastungen;

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

function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct, A_fun, B_fun,pnum)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,pnum);

x_dot = PSA_dynamics_codegen(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);
A_jacobian = A_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis,p_struct.g);
B_jacobian = B_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);

S_dot_matrix = zeros(7,pnum);
for j = 1:pnum
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot; S_dot_matrix(:)];
disp(t)
end






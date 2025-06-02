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

%% Syms

syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
syms Ue1 Ue2 real
syms p_Motoruebersetzung p_motor_traegheit p_m_ges p_Reifen_Radius p_I_Bot p_Achsabstand p_motor_Drehmomentkoef p_motor_Daempfung p_motor_Widerstand p_motor_Induktivitaet p_motor_BackEMFkoef p_I_Reifen p_mue_g p_Xi p_B_dis p_g real

x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
u_sym = [Ue1; Ue2];
params_sym = [p_Motoruebersetzung; p_motor_traegheit; p_m_ges; p_Reifen_Radius; p_I_Bot; p_Achsabstand; p_motor_Drehmomentkoef; p_motor_Daempfung; p_motor_Widerstand; p_motor_Induktivitaet; p_motor_BackEMFkoef; p_I_Reifen; p_mue_g; p_Xi; p_B_dis; p_g];
params_sym_ordered = {p_Motoruebersetzung, p_motor_traegheit, p_m_ges, p_Reifen_Radius, p_I_Bot, p_Achsabstand, p_motor_Drehmomentkoef, p_motor_Daempfung, p_motor_Widerstand, p_motor_Induktivitaet, p_motor_BackEMFkoef, p_I_Reifen,p_mue_g, p_Xi, p_B_dis, p_g};

%% xdot-symbolic
J_Mn_sym = p_Motoruebersetzung^2 * p_motor_traegheit;
C1_sym = (p_m_ges*p_Reifen_Radius^2)/4 - (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);
C2_sym = (p_m_ges*p_Reifen_Radius^2)/4 + (p_I_Bot*p_Reifen_Radius^2)/(p_Achsabstand^2);

M11_sym = J_Mn_sym + C1_sym + p_I_Reifen;
M12_sym = C2_sym;
M21_sym = M12_sym;
M22_sym = M11_sym;

J_g_sym = [M11_sym M12_sym;
           M21_sym M22_sym];

v_Tresh_sym = 1e-3;
k_smooth_sym = 5000;

eps_om_sym = 1e-6;
eta_abs_sq_sym = (eps_om_sym/10)^2;
k_xp_sym = 5 / eps_om_sym;
delta_sig_sq_sym = 1e-6;

OmegaSum_sym = om_m1 + om_m2;
v_Bot_sym = (p_Reifen_Radius/2)*OmegaSum_sym;
smooth_factor_sym = 0.5 * (1 + tanh(k_smooth_sym * (v_Bot_sym - v_Tresh_sym)));
C_Frb_sym = p_mue_g * p_m_ges * p_g * (p_Xi/2);
Frb_smooth_sym = smooth_factor_sym * C_Frb_sym;

deltaOmega_sym = om_m2-om_m1;
smooth_abs_DeltaOmega_sym = sqrt(deltaOmega_sym^2 + eta_abs_sq_sym);
w_sym = 0.5 * (1 + tanh(k_xp_sym*(smooth_abs_DeltaOmega_sym - eps_om_sym)));

inv_xp_A_const_sym = 1e-6;

inv_xp_B_turn_nun_sym = (2/p_Achsabstand) * deltaOmega_sym * OmegaSum_sym;
inv_xp_B_turn_dun_sym = OmegaSum_sym^2 + delta_sig_sq_sym;
inv_xp_B_turn_sym = inv_xp_B_turn_nun_sym / inv_xp_B_turn_dun_sym;

inv_xp_smooth_sym = (1-w_sym) * inv_xp_A_const_sym + w_sym * inv_xp_B_turn_sym;

frac_smooth_sym = p_B_dis * inv_xp_smooth_sym;

nenner_sc_alpha_sym = sqrt(1+ frac_smooth_sym^2);
salpha_smooth_sym = 1 / nenner_sc_alpha_sym;
calpha_smooth_sym = frac_smooth_sym / nenner_sc_alpha_sym;

C_Tr_sym = (2 * p_Reifen_Radius/p_Achsabstand) * (1-p_Xi) * p_B_dis;

Tr1_smooth_sym = -Frb_smooth_sym * (p_Reifen_Radius * salpha_smooth_sym + C_Tr_sym * calpha_smooth_sym);
Tr2_smooth_sym = -Frb_smooth_sym * (p_Reifen_Radius * salpha_smooth_sym - C_Tr_sym * calpha_smooth_sym);

M_belastungen_sym = [p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m1 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m1+ Tr1_smooth_sym;
                     p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m2 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m2+ Tr2_smooth_sym];

om_dot_sym = inv(J_g_sym) * M_belastungen_sym;

di1dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m1 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m1 + (1/p_motor_Induktivitaet)* Ue1;
dom1dt_sym = om_dot_sym(1);
di2dt_sym = -(p_motor_Widerstand/p_motor_Induktivitaet) * i_m2 - (p_Motoruebersetzung * p_motor_BackEMFkoef/p_motor_Induktivitaet) * om_m2 + (1/p_motor_Induktivitaet)* Ue2;
dom2dt_sym = om_dot_sym(2);
vx_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
vy_sym = (p_Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
dthetadt_sym = (p_Reifen_Radius/p_Achsabstand) * (om_m2-om_m1);

xdot_sym = [di1dt_sym;dom1dt_sym;di2dt_sym;dom2dt_sym;vx_sym;vy_sym;dthetadt_sym];

%% A and B matrix

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

S0 = zeros(7,16);
X_aug_0 = [x0;S0(:)];

%% ODE-Sim

options = odeset('RelTol',1e-6,'AbsTol',1e-8);
[T,X_aug_sol] = ode45(@(t, X_aug) AugmentedDynamics(t, X_aug, U_sim, p_struct, A_fun, B_fun), t_span, X_aug_0, options);

x_traj = X_aug_sol(:,1:7);
S_sol = X_aug_sol(:,8:end);

num_t_steps = length(T);
S_traj_tensor = zeros(num_t_steps,7,16);
for i = 1:num_t_steps
    S_traj_tensor(i,:,:) = reshape(S_sol(i,:), 7, 16);
end

%% plot-solutions - relative Sensitivities
p_list = fieldnames(p_struct);
p_nom_values = zeros(length(p_list),1);
for k = 1:length(p_nom_values)
    p_nom_values(k) = p_struct.(p_list{k});
end
s_list = {'i_m1','om_m1','i_m2','om_m2','xpos','ypos','theta'};
s_list_latex = {'$i_{M1}$','$\omega_{M1}$','$i_{M2}$','$\omega_{M2}$','$x_{pos}$','$y_{pos}$','$\theta$'};
p_list_latex = {'Motoruebersetzung', '$Traegheit_M$', '$m_{ges}$', '$R_{Reifen}$', '$I_{Bot}$', 'Achsabstand', '$Drehmomentkoef_M$', '$Daempfung_M$', '$\Omega_M$', '$Induktivitaet_M$', '$BackEMFkoef_M$', '$I_{Reifen}$', '$\mu_g$', '$Xi$', '$B_{dis}$', 'g'};

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

function xdot = Dynamics(x,u,Motoruebersetzung, motor_traegheit, m_ges, Reifen_Radius, I_Bot, Achsabstand, motor_Drehmomentkoef, motor_Daempfung,motor_Widerstand, motor_Induktivitaet, motor_BackEMFkoef,I_Reifen,mue_g,  Xi, B_dis,g)
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

v_Tresh = 1e-3;
k_smooth = 5000;

eps_om = 1e-6;
eta_abs_sq = (eps_om/10)^2;
k_xp = 5 / eps_om;
delta_sig_sq = 1e-6;

OmegaSum = om_m1 + om_m2;
v_Bot = (Reifen_Radius/2)*OmegaSum;
smooth_factor = 0.5 * (1 + tanh(k_smooth * (v_Bot - v_Tresh)));
C_Frb = mue_g * m_ges * g * (Xi/2);
Frb_smooth = smooth_factor * C_Frb;

deltaOmega = om_m2-om_m1;
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

M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * i_m1 - Motoruebersetzung^2 * motor_Daempfung*om_m1 + Tr1_smooth;
                 Motoruebersetzung * motor_Drehmomentkoef * i_m2 - Motoruebersetzung^2 * motor_Daempfung*om_m2 + Tr2_smooth];

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

function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct, A_fun, B_fun)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,16);

x_dot = Dynamics(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);
A_jacobian = A_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis,p_struct.g);
B_jacobian = B_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);

S_dot_matrix = zeros(7,16);
for j = 1:16
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot; S_dot_matrix(:)];
end




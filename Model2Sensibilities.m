clc; clear; close all; 
% Modell 2 Sensibilitäten-> ohne Schleifblöcke

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

%% Syms

syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
syms Ue1 Ue2 real
syms p_Motoruebersetzung p_motor_traegheit p_m_ges p_Reifen_Radius p_I_Bot p_Achsabstand p_motor_Drehmomentkoef p_motor_Daempfung p_motor_Widerstand p_motor_Induktivitaet p_motor_BackEMFkoef p_I_Reifen real

x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
u_sym = [Ue1; Ue2];
params_sym = [p_Motoruebersetzung; p_motor_traegheit; p_m_ges; p_Reifen_Radius; p_I_Bot; p_Achsabstand; p_motor_Drehmomentkoef; p_motor_Daempfung; p_motor_Widerstand; p_motor_Induktivitaet; p_motor_BackEMFkoef; p_I_Reifen];
params_sym_ordered = {p_Motoruebersetzung, p_motor_traegheit, p_m_ges, p_Reifen_Radius, p_I_Bot, p_Achsabstand, p_motor_Drehmomentkoef, p_motor_Daempfung, p_motor_Widerstand, p_motor_Induktivitaet, p_motor_BackEMFkoef, p_I_Reifen};

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

M_belastungen_sym = [p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m1 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m1;
                 p_Motoruebersetzung * p_motor_Drehmomentkoef * i_m2 - p_Motoruebersetzung^2 * p_motor_Daempfung*om_m2];

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

S0 = zeros(7,12);
X_aug_0 = [x0;S0(:)];

%% ODE-Sim

options = odeset('RelTol',1e-6,'AbsTol',1e-8);
[T,X_aug_sol] = ode45(@(t, X_aug) AugmentedDynamics(t, X_aug, U_sim, p_struct, A_fun, B_fun), t_span, X_aug_0, options);

x_traj = X_aug_sol(:,1:7);
S_sol = X_aug_sol(:,8:end);

num_t_steps = length(T);
S_traj_tensor = zeros(num_t_steps,7,12);
for i = 1:num_t_steps
    S_traj_tensor(i,:,:) = reshape(S_sol(i,:), 7, 12);
end

%% plot-solutions - relative Sensitivities
p_list = fieldnames(p_struct);
p_nom_values = zeros(length(p_list),1);
for k = 1:length(p_nom_values)
    p_nom_values(k) = p_struct.(p_list{k});
end
s_list = {'i_m1','om_m1','i_m2','om_m2','xpos','ypos','theta'};
s_list_latex = {'$i_{M1}$','$\omega_{M1}$','$i_{M2}$','$\omega_{M2}$','$x_{pos}$','$y_{pos}$','$\theta$'};
p_list_latex = {'Motoruebersetzung', '$Traegheit_M$', '$m_{ges}$', '$R_{Reifen}$', '$I_{Bot}$', 'Achsabstand', '$Drehmomentkoef_M$', '$Daempfung_M$', '$\Omega_M$', '$Induktivitaet_M$', '$BackEMFkoef_M$', '$I_{Reifen}$'};

S_rel_tensor = zeros(size(S_traj_tensor));
for kp = 1:length(p_list)
    S_rel_tensor(:,:,kp) = S_traj_tensor(:,:,kp) * p_nom_values(kp);
end

%plot every relative Sensibilities alone
% again = true;
% counter = 0;
% while again == true
%     while true
%         if counter ~=0
%             stop_input = input([newline 'another plot? (Y / N)'], 's');
%             if strcmpi(stop_input, "Y")
%                 close all;
%                 break;
%             elseif strcmpi(stop_input, "N")
%                 again = false;
%                 break;
%             else
%                 disp('input not valid');
%             end
%         else
%             break
%         end
%     end
% 
%     if ~again
%         break;
%     end
% 
%     while true
%         state_input = input([newline 'chose interested state:' newline 'i_m1-> 1' newline 'om_m1-> 2' newline 'i_m2-> 3' newline 'om_m2-> 4' newline 'xpos-> 5' newline 'ypos-> 6' newline 'theta-> 7' newline 'stop plotting-> End' newline 'input:'], 's');
%         if strcmpi(state_input, "End")
%             again = false;
%             break;
%         end
%         state_val = str2double(state_input);
%         if ~isnan(state_val) && mod(state_val, 1) == 0 && state_val >= 1 && state_val <= 7
%             state_idx = state_val;
%             break;
%         else
%             disp('input not valid');
%         end
%     end
% 
%     if ~again
%         break;
%     end
% 
%     while true
%         param_input = input([newline 'chose interested parameter:' newline 'Motoruebersetzung-> 1' newline 'motor_traegheit-> 2' newline 'm_ges-> 3' newline 'Reifen_Radius-> 4' newline 'I_Bot-> 5' newline 'Achsabstand-> 6' newline 'motor_Drehmomentkoef-> 7' newline 'motor_Daempfung-> 8' newline 'motor_Widerstand-> 9' newline 'motor_Induktivitaet-> 10' newline 'motor_BackEMFkoef-> 11' newline 'I_Reifen-> 12' newline 'stop plotting-> End' newline 'input:'], 's');
%         if strcmpi(param_input, "End")
%             again = false;
%             break;
%         end
% 
%         param_val = str2double(param_input);
%         if ~isnan(param_val) && mod(param_val, 1) == 0 && param_val>= 1 && param_val <= 12
%             param_idx = param_val;
%             break;
%         else
%             disp('input not valid');
%         end
%     end
% 
%     if ~again
%         break;
%     end
% 
%     counter = counter+1;
%     figure;
%     plot(T,S_rel_tensor(:,state_idx,param_idx))
%     title(['relative sensitvity (p_k*dS/dp_k) of', s_list{state_idx},'to',p_list{param_idx}]);
%     xlabel('time in [s]');
%     ylabel('(p_k*dS/dp_k)')
%     grid on;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct, A_fun, B_fun)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,12);

x_dot = Dynamics(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen);
A_jacobian = A_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen);
B_jacobian = B_fun(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen);

S_dot_matrix = zeros(7,12);
for j = 1:12
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot; S_dot_matrix(:)];
end




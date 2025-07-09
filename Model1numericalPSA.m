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

% Motormodell

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

%codegen PSA_dynamics_codegen -args ARGS -report
codegen('PSA_dynamics_codegen', '-args', ARGS, '-report');

disp('Codegen abgeschlossen. Starte ODE-Simulation...');

pnum = length(ARGS)-2;

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
A_handle = @(t, x) A_fun(x, U_sim, p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen,p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g, p_struct.L_B);
options = odeset('RelTol',1e-3, 'Jacobian', A_handle, 'JPattern', J_pattern);%,'AbsTol',auto
[T,X_aug_sol] = ode15s(@(t, X_aug) AugmentedDynamics(t, X_aug, U_sim, p_struct, pnum), t_span, X_aug_0, options);

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

%% Augmented Dynamics Function

function dX_aug_dt = AugmentedDynamics(t, X_aug, u, p_struct,pnum)
x = X_aug(1:7);
S_matrix = reshape(X_aug(8:end),7,pnum);

dynamics_handle = @PSA_dynamics_codegen; 

x_dot = dynamics_handle(x,u,p_struct.Motoruebersetzung, p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung,p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef,p_struct.I_Reifen, p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);
A_jacobian = compute_A_numerical(dynamics_handle, x, u, p_struct);
B_jacobian = compute_B_numerical(dynamics_handle, x, u, p_struct, p_list);

S_dot_matrix = zeros(7,pnum);
for j = 1:pnum
    S_j = S_matrix(:,j);
    df_dpj = B_jacobian(:,j);
    S_dot_matrix(:,j) = A_jacobian * S_j + df_dpj;
end
dX_aug_dt = [x_dot; S_dot_matrix(:)];
disp(t)
end

% HILFSFUNKTION 1: Berechnet die Jacobi-Matrix A numerisch
function A = compute_A_numerical(dynamics_handle, x, u, p_struct)
    n = length(x);
    A = zeros(n, n);
    delta = 1e-7; % Kleine Störung

    fx = dynamics_handle(x, u, p_struct.Motoruebersetzung, p_struct.motor_traegheit, ... 
    p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, ...
    p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, ...
    p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, ...
    p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);

    for j = 1:n
        x_perturbed = x;
        x_perturbed(j) = x_perturbed(j) + delta;
        
        fx_perturbed = dynamics_handle(x_perturbed, u, p_struct.Motoruebersetzung, ...
        p_struct.motor_traegheit, p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, ...
        p_struct.Achsabstand, p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, ...
        p_struct.motor_Widerstand, p_struct.motor_Induktivitaet, ...
        p_struct.motor_BackEMFkoef, p_struct.I_Reifen, p_struct.mue_g, ...
        p_struct.Xi, p_struct.B_dis, p_struct.g);
        
        A(:, j) = (fx_perturbed - fx) / delta;
    end
end


% HILFSFUNKTION 2: Berechnet die Jacobi-Matrix B (Sensitivität zu Parametern) numerisch
function B = compute_B_numerical(dynamics_handle, x, u, p_struct, p_list)
    n = length(x);
    p_num = length(p_list);
    B = zeros(n, p_num);
    delta_rel = 1e-7; % Relative Störung

    fx = dynamics_handle(x, u, p_struct.Motoruebersetzung, p_struct.motor_traegheit, ...
    p_struct.m_ges, p_struct.Reifen_Radius, p_struct.I_Bot, p_struct.Achsabstand, ...
    p_struct.motor_Drehmomentkoef, p_struct.motor_Daempfung, p_struct.motor_Widerstand, ...
    p_struct.motor_Induktivitaet, p_struct.motor_BackEMFkoef, p_struct.I_Reifen, ...
    p_struct.mue_g, p_struct.Xi, p_struct.B_dis, p_struct.g);

    for j = 1:p_num
        p_struct_perturbed = p_struct;
        param_name = p_list{j};
        original_val = p_struct_perturbed.(param_name);
        delta = delta_rel * original_val;
        if delta == 0; delta = delta_rel; end
        
        p_struct_perturbed.(param_name) = original_val + delta;
        
        fx_perturbed = dynamics_handle(x, u, p_struct_perturbed.Motoruebersetzung, ...
        p_struct_perturbed.motor_traegheit, p_struct_perturbed.m_ges, ...
        p_struct_perturbed.Reifen_Radius, p_struct_perturbed.I_Bot, ...
        p_struct_perturbed.Achsabstand, p_struct_perturbed.motor_Drehmomentkoef, ...
        p_struct_perturbed.motor_Daempfung, p_struct_perturbed.motor_Widerstand, ...
        p_struct_perturbed.motor_Induktivitaet, p_struct_perturbed.motor_BackEMFkoef, ...
        p_struct_perturbed.I_Reifen, p_struct_perturbed.mue_g, p_struct_perturbed.Xi, ...
        p_struct_perturbed.B_dis, p_struct_perturbed.g);
        
        B(:, j) = (fx_perturbed - fx) / delta;
    end
end


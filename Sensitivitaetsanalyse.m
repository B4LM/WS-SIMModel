clc; clear; close all; 
%für beide Modelle

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



%% Sim Parameters
t_span = [0 1.5];            % [s]
StartPos = [0.3;0.3];        % [m]
U_sim = [10.5; 12];          % [V]

ParamsList= {'g','m_ges','Achsabstand','Motoruebersetzung','B_dis','Xi','I_Bot','mue_g','Reifen_Radius','I_Reifen','L_B','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef'};


pstruct = struct();

%% Startwerte
x0 = [0;
       0;
       0;
       0;
       StartPos(1);
       StartPos(2);
       0;];

for n=1:length(ParamsList)
    pname = ParamsList{n};
    pstruct.(pname) = Sensibilities(eval(pname));
end
%Paramfielnames = fieldnames(pstruct);

for i = 1:length(ParamsList)
    
    for j=1:5
        ParamZero = eval(ParamsList{i});
        switch j
            case 1
                factor = 0.9;
            case 2
                factor = 0.95;
            case 3
                factor = 1;
            case 4
                factor = 1.05;
            case 5
                factor = 1.1;
        end
        assignin('base',ParamsList{i},pstruct.(ParamsList{i}).value * factor)
        %disp(eval(ParamsList{i}))
        %     end
        %     assignin('base',ParamsList{i},pstruct.(ParamsList{i}).value)
        % end
      

        %Simulationsvorbereitung
    
        syms i_m1 om_m1 i_m2 om_m2 xpos ypos theta real
        syms Ue1 Ue2 real
    
        x_sym = [i_m1; om_m1;i_m2; om_m2; xpos; ypos; theta];
        u_sym = [Ue1; Ue2];
        z_sym = sym('z',[1 1]);
        y_sym = sym('y',[1 1]);
        
        %% Function
        %x-vec:
        i_m1 = x_sym(1);
        om_m1 = x_sym(2);
        i_m2 = x_sym(3);
        om_m2 = x_sym(4);
        theta = x_sym(7);
        
        %Spannungs-Eingang
        Ue1 = u_sym(1);
        Ue2 = u_sym(2);
        
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
        
        f1_sym = -(motor_Widerstand/motor_Induktivitaet) * i_m1 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m1 + (1/motor_Induktivitaet)* Ue1;
        f2_sym = om_dot(1);
        f3_sym = -(motor_Widerstand/motor_Induktivitaet) * i_m2 - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * om_m2 + (1/motor_Induktivitaet)* Ue2;
        f4_sym = om_dot(2);
        f5_sym = (Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
        f6_sym = (Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
        f7_sym = (Reifen_Radius/Achsabstand) * (om_m2-om_m1);
    
       
        f_sym = [
        f1_sym;
        f2_sym;
        f3_sym;
        f4_sym;
        f5_sym;
        f6_sym;
        f7_sym;
        ];
    
        g_sym = [
        x_sym(5);
        x_sym(6);
        ];
    
        x0s = [0;
           0;
           0;
           0;
           StartPos(1);
           StartPos(2);
           0;];
    
        u0 = [
            0;
            0;
            ];
    
        z0 = 0;
    
        A_sym = jacobian(f_sym,x_sym);
        b_sym = jacobian(f_sym,u_sym);
    
        c_sym = jacobian(g_sym,x_sym);
    
        A_func = matlabFunction(A_sym,'Vars',{x_sym,u_sym,z_sym});
        b_func = matlabFunction(b_sym,'Vars',{x_sym,u_sym,z_sym});
    
        c_func = matlabFunction(c_sym,'Vars',{x_sym,u_sym,z_sym});
    
        A = A_func(x0s,u0,z0);
        b = b_func(x0s,u0,z0);
    
        c = c_func(x0s,u0,z0);
        
        d= 0;
        sys = ss(A, b, c, d);
    
        %% Simulation selber
        
        disp('Sim startet!')
        [t, x] = ode15s(@(t_arg, x_arg) ss_ode(t_arg, x_arg, sys.A, sys.B, U_sim), t_span, x0);
        disp('Sim beendet!')
        
    
        %Results
        pos = [x(5),x(6)];

        vel = zeros(size(x));
        for i = 1:length(t)
            dxdt_i = ss_ode(t(i), x(i,:)', sys.A, sys.B, U_sim);
            vel(i,:) = dxdt_i';
        end 

        pstruct.(ParamsList{i}).savePosData(pos,j);
        pstruct.(ParamsList{i}).saveVelData(vel,j);
        pstruct.(ParamsList{i}).saveTime(t,j);
        %pstruct.(ParamsList{i}).position(j) = [out.position.Data(:,1), out.position.Data(:,2)];
        %pstruct.(ParamsList{i}).velocity(j) = [out.velocity.Time, out.velocity.Data];
    end
    assignin('base',ParamsList{i},pstruct.(ParamsList{i}).value)
end

for pn = 1: length(ParamsList)
    plotSensibilities(pstruct,ParamsList(pn))
    input('nächter Parameter');
end

function dxdt = ss_ode(t, x, A, B, u)
    dxdt = A*x + B*u;
end


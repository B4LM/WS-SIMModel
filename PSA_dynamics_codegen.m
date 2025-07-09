function xdot = PSA_dynamics_codegen(x,u,p)
%#codegen

%x-vec:
i_m1 = x(1);
om_m1 = x(2);
i_m2 = x(3);
om_m2 = x(4);
theta = x(7);

%Spannungs-Eingang
Ue1 = u(1);
Ue2 = u(2);

J_Mn = p.Motoruebersetzung^2 * p.motor_traegheit;
C1 = (p.m_ges*p.Reifen_Radius^2)/4 - (p.I_Bot*p.Reifen_Radius^2)/(p.Achsabstand^2);
C2 = (p.m_ges*p.Reifen_Radius^2)/4 + (p.I_Bot*p.Reifen_Radius^2)/(p.Achsabstand^2);

M11 = J_Mn + C1 + p.I_Reifen;
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
B1 = [(1-p.Xi)*p.B_dis; p.L_B/2];
B2 = [(1-p.Xi)*p.B_dis; -p.L_B/2];

% Berechnung der reibungskraft-Angriffswinkel über Ermittlung von
% Geschwindigkeitspol xp
OmegaSum = om_m1 + om_m2;
deltaOmega = om_m2-om_m1;
smooth_abs_DeltaOmega = sqrt(deltaOmega^2 + eta_abs_sq);

v_Bot = (p.Reifen_Radius/2)*OmegaSum;
omega_Bot = (p.Reifen_Radius/p.Achsabstand)*smooth_abs_DeltaOmega;

% Glättungsfunktionen für Reibungskraft-> keine Reibung bei v_Bot / omega_Bot =0
smooth_factor_straight = 0.5 * (1 + tanh(k_smooth * (v_Bot - v_Tresh)));
smooth_factor_spin = 0.5 * (1 + tanh(k_smooth * (omega_Bot - omega_Tresh)));
C_Frb = p.mue_g * p.m_ges * p.g * (p.Xi/2);
Frb_smooth_straight = smooth_factor_straight * C_Frb;
Frb_smooth_spin = smooth_factor_spin * C_Frb;

% Bestimmung von Geschwndigkeitspol xp
w = 0.5 * (1 + tanh(k_xp*(smooth_abs_DeltaOmega - eps_om)));

inv_xp_A_const = 1e-6;

inv_xp_B_turn_nun = (2/p.Achsabstand) * deltaOmega * OmegaSum;
inv_xp_B_turn_dun = OmegaSum^2 + delta_sig_sq;
inv_xp_B_turn = inv_xp_B_turn_nun / inv_xp_B_turn_dun;

inv_xp_smooth = (1-w) * inv_xp_A_const + w * inv_xp_B_turn;
xp_smooth = 1./(inv_xp_smooth+xp_delta);

xppos_translation = [-p.Xi*p.B_dis; xp_smooth];
xppos_turnonpint = [-p.Xi*p.B_dis; 0];

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
Tr1_smooth = -(Frb_smooth_straight* p.Reifen_Radius/2) * (Fb1_dir_unit(1) + Fb2_dir_unit(1)) + w*(p.Reifen_Radius/p.Achsabstand)*(M_FR1+M_FR2);
Tr2_smooth = -(Frb_smooth_straight* p.Reifen_Radius/2) * (Fb1_dir_unit(1) + Fb2_dir_unit(1)) - w*(p.Reifen_Radius/p.Achsabstand)*(M_FR1+M_FR2);

% Gesamte Belastungen an den Motoren
M_belastungen = [p.Motoruebersetzung * p.motor_Drehmomentkoef * i_m1 - p.Motoruebersetzung^2 * p.motor_Daempfung*om_m1 + Tr1_smooth;
                 p.Motoruebersetzung * p.motor_Drehmomentkoef * i_m2 - p.Motoruebersetzung^2 * p.motor_Daempfung*om_m2 + Tr2_smooth];

% Bestimmung von Winkelbeschleunigungen der Motoren
om_dot = J_g \ M_belastungen;

di1dt = -(p.motor_Widerstand/p.motor_Induktivitaet) * i_m1 - (p.Motoruebersetzung * p.motor_BackEMFkoef/p.motor_Induktivitaet) * om_m1 + (1/p.motor_Induktivitaet)* Ue1;
dom1dt = om_dot(1);
di2dt = -(p.motor_Widerstand/p.motor_Induktivitaet) * i_m2 - (p.Motoruebersetzung * p.motor_BackEMFkoef/p.motor_Induktivitaet) * om_m2 + (1/p.motor_Induktivitaet)* Ue2;
dom2dt = om_dot(2);
vx = (p.Reifen_Radius/2) * (om_m1 + om_m2) * cos(theta);
vy = (p.Reifen_Radius/2) * (om_m1 + om_m2) * sin(theta);
dthetadt = (p.Reifen_Radius/p.Achsabstand) * (om_m2-om_m1);

xdot = [di1dt;dom1dt;di2dt;dom2dt;vx;vy;dthetadt];

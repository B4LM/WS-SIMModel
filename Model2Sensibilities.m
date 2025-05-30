clc; clear; close all; 
% Modell 2 Sensibilitäten-> ohne Schleifblöcke
%% Wegpunkte

StartPos = [0.3;0.3];     
Zielpunkt = [1;1.2];

%% general

g = 9.81;                    % [m/s^2]

P.m_ges = 1;                   % [kg]
P.m_last = 0.424;              % [kg]
P.Achsabstand = 0.15;          % [m]    
P.Motoruebersetzung = 100;     % [1]
P.B_dis = 0.25;                % [m]
P.R_dis = 0.15;                % [m]
P.Xi = 2/3;                    % [1]
P.I_Bot = 0.0072;              % [kg*m^2]
P.mue_g = 0.1;                 % [1]
P.Reifen_Radius = 0.04;        % [m]


%% Motormodell

P.motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
P.motor_Induktivitaet = 0.0002;       % [H]
P.motor_Widerstand =  13.33;          % [Ohm]
P.motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
P.motor_Drehmomentkoef = 0.0174;      % [N*m / A]
P.motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%'m_ges','Achsabstand', 'B_dis','R_dis'
%,'Xi','I_Bot','mue_g','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef','m_Reifen','Reifen_Radius',
%'Achsabstand','Motoruebersetzung'
PListges = {'m_ges','Achsabstand', 'B_dis','R_dis','Xi','I_Bot','mue_g','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef','m_Reifen','Reifen_Radius','Motoruebersetzung'};
numPListges = length(PListges);
% PsenList= {'m_ges','Achsabstand'};
% numPsenList = length(PsenList);

x_sym = sym('x',[7 1],'real');
u_sym = sym('u',[2 1],'real');

params_sym = cell(1,numPListges);
for k = 1:numPListges
    params_sym{k} = sym([PListges{k},'_s'],'real');
end
disp(params_sym)

%% Diff-Gleichungen


J_Mn = params_sym{16}^2 * params_sym{8};
C1 = (params_sym{1}*params_sym{15}^2)/4 - (params_sym{6}*params_sym{15}^2)/(params_sym{2}^2);
C2 = (params_sym{1}*params_sym{15}^2)/4 + (params_sym{6}*params_sym{15}^2)/(params_sym{2}^2);

M11 = J_Mn + C1;
M12 = C2;
M21 = M12;
M22 = M11;

J_g = [M11 M12;
       M21 M22];

M_belastungen = [params_sym{16} * params_sym{12} * x_sym(1) - params_sym{16}^2 * params_sym{11}*x_sym(2);
                 params_sym{16} * params_sym{12} * x_sym(3) - params_sym{16}^2 * params_sym{11}*x_sym(4)];

om_dot = inv(J_g) * M_belastungen;

di1dt = -(params_sym{10}/params_sym{9}) * x_sym(1) - (params_sym{16} * params_sym{13}/params_sym{9}) * x_sym(2) + (1/params_sym{9})* u_sym(1);
dom1dt = om_dot(1);
di2dt = -(params_sym{10}/params_sym{9}) * x_sym(3) - (params_sym{16} * params_sym{13}/params_sym{9}) * x_sym(4) + (1/params_sym{9})* u_sym(2);
dom2dt = om_dot(2);
vx = (params_sym{15}/2) * (x_sym(2) + x_sym(4)) * cos(x_sym(7));
vy = (params_sym{15}/2) * (x_sym(2) + x_sym(4)) * sin(x_sym(7));
dthetadt = (params_sym{15}/params_sym{2}) * (x_sym(4)-x_sym(2));

f_sym = [di1dt;
        dom1dt;
        di2dt;
        dom2dt;
        vx;
        vy;
        dthetadt];


Sx = jacobian(f_sym,params_sym);
Sx = subs(Sx,params_sym{:}, P.(PListges{:}));

A_sym = jacobian(f_sym,x_sym);
vars_list_Afunc = {x_sym,u_sym,[params_sym{:}]};
A_func = matlabFunction(A_sym, 'Vars',vars_list_Afunc);




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




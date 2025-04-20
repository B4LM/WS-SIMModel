clc; clear; close all; 

%% Wegpunkte

StartPos = [0.3;0.3];     
Zielpunkt = [1;1.2];

%% general

g = 9.81;                    % [m/s^2]
m_ges = 1;                   % [kg]
m_last = 0.424;              % [kg]
Achsabstand = 0.15;          % [m]    
Motoruebersetzung = 100;     % [1]
B_dis = 0.25;                % [m]
R_dis = 0.15;                % [m]
Xi = 2/3;                    % [1]
I_Bot = 0.0072;              % [kg*m^2]
mue_g = 0.1;                 % [1]


%% Motormodell

motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
motor_Drehmomentkoef = 0.0174;      % [N*m / A]
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% Reifenmodell

m_Reifen = 0.038;                % [kg]               
Reifen_Radius = 0.04;            % [m]

Params = [motor_Widerstand ; motor_Induktivitaet; motor_BackEMFkoef; motor_Drehmomentkoef; motor_traegheit; motor_Daempfung];

%% symbolische Variablen

x_sym = sym('x',[2 1]);
u_sym = sym('u',[2 1]);
z_sym = sym('z',[1 1]);
y_sym = sym('y',[1 1]);
p_sym = sym('p',[6,1]);


%% symbolische Funktionen

% Momentengleichgewicht im Motor (Last als Eingangsgröße)
f1_sym = -(p_sym(1)/p_sym(2)) * x_sym(1) - (p_sym(3)/p_sym(2)) * x_sym(2)  + (1/p_sym(2))* u_sym(1);
% Maschenregel in Motor
f2_sym = (p_sym(4)/p_sym(5)) * x_sym(1)- (p_sym(6)/p_sym(5)) * x_sym(2)  - (1/p_sym(5))*u_sym(2);

fmatrix = [f1_sym; f2_sym];
dfdx = jacobian(fmatrix, x_sym);
%dfdx_0 = subs(dfdx,p_sym,Params);
dfdp = jacobian(fmatrix,p_sym);
%dfdp_0 = subs(dfdp,p_sym,Params);
Sx = jacobian(x_sym,p_sym);

x0s = [
    0;
    0;
    ];

u0 = [
    0;
    0;
    ];

z0 = 0;


TSmatrix = dfdx_0 * Sx + dfdp_0;
T_func = matlabFunction(TSmatrix,'Vars',{x_sym,u_sym,z_sym});
T = T_func(x0s,u0,z0)







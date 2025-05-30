clc; clear; close all; 
% Modell 2 -> ohne Schleifblöcke
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
Reifen_Radius = 0.04;            % [m]


%% Motormodell

motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4];
motor_Drehmomentkoef = 0.0174;      % [N*m / A]
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%% symbolische Variablen

x_sym = sym('x',[7 1]);
u_sym = sym('u',[2 1]);
z_sym = sym('z',[1 1]);
y_sym = sym('y',[1 1]);


%% symbolische Funktionen Modell-Groß
% Anpassungen wegen Motorübersetzung
J_Mn = Motoruebersetzung^2 * motor_traegheit;
C1 = (m_ges*Reifen_Radius^2)/4 - (I_Bot*Reifen_Radius^2)/(Achsabstand^2);
C2 = (m_ges*Reifen_Radius^2)/4 + (I_Bot*Reifen_Radius^2)/(Achsabstand^2);

M11 = J_Mn + C1;
M12 = C2;
M21 = M12;
M22 = M11;

J_g = [M11 M12;
       M21 M22];

M_belastungen = [Motoruebersetzung * motor_Drehmomentkoef * x_sym(1) - Motoruebersetzung^2 * motor_Daempfung*x_sym(2);
                 Motoruebersetzung * motor_Drehmomentkoef * x_sym(3) - Motoruebersetzung^2 * motor_Daempfung*x_sym(4)];

om_dot = inv(J_g) * M_belastungen;

f1_sym = -(motor_Widerstand/motor_Induktivitaet) * x_sym(1) - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * x_sym(2) + (1/motor_Induktivitaet)* u_sym(1);
f2_sym = om_dot(1);
f3_sym = -(motor_Widerstand/motor_Induktivitaet) * x_sym(3) - (Motoruebersetzung * motor_BackEMFkoef/motor_Induktivitaet) * x_sym(4) + (1/motor_Induktivitaet)* u_sym(2);
f4_sym = om_dot(2);
f5_sym = (Reifen_Radius/2) * (x_sym(2) + x_sym(4)) * cos(x_sym(7));
f6_sym = (Reifen_Radius/2) * (x_sym(2) + x_sym(4)) * sin(x_sym(7));
f7_sym = (Reifen_Radius/Achsabstand) * (x_sym(4)-x_sym(2));

%% state functions
f_sym = [
    f1_sym;
    f2_sym;
    f3_sym;
    f4_sym;
    f5_sym;
    f6_sym;
    f7_sym;
    ];

%% output function
g_sym = [
    x_sym(5);
    x_sym(6);
    x_sym(7);
    ];

%% initial values
x0s = [
    0;
    0;
    0;
    0;
    StartPos(1);
    StartPos(2);
    0;
    ];

u0 = [
    0;
    0;
    ];

z0 = 0;

%% system matrices (symbolic)

A_sym = jacobian(f_sym,x_sym);
b_sym = jacobian(f_sym,u_sym);
e_sym = jacobian(f_sym,z_sym);

c_sym = jacobian(g_sym,x_sym);

%% system matrices (functions)

A_func = matlabFunction(A_sym,'Vars',{x_sym,u_sym,z_sym});
b_func = matlabFunction(b_sym,'Vars',{x_sym,u_sym,z_sym});
e_func = matlabFunction(e_sym,'Vars',{x_sym,u_sym,z_sym});

c_func = matlabFunction(c_sym,'Vars',{x_sym,u_sym,z_sym});

%% system matrices (numerical)

A = A_func(x0s,u0,z0);
b = b_func(x0s,u0,z0);
e = e_func(x0s,u0,z0);

c = c_func(x0s,u0,z0);
d = 0;


%% state space system

sys = ss(A,b,c,d);

%% Map

myMap = binaryOccupancyMap(1.8,1.8,100);

walls = zeros(180,180);
walls(15,15:165) = 1; % Nord-Wand
walls(165,15:165) = 1; % Süd-Wand
walls(15:165,15) = 1; % West-Wand
walls(15:165,165) = 1; % Ost-Wand
walls((180-Zielpunkt(2)*100)-1:(180-Zielpunkt(2)*100)+1,(Zielpunkt(1)*100)-1:(Zielpunkt(1)*100)+1) = 1; %Zielpunkt


setOccupancy(myMap,[1 1], walls, "grid")
show(myMap)









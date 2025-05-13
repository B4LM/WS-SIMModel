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

%% symbolische Variablen

x_sym = sym('x',[2 1]);
u_sym = sym('u',[2 1]);
z_sym = sym('z',[1 1]);
y_sym = sym('y',[1 1]);

xk_sym = sym('x',[4 1]);
uk_sym = sym('u',[2 1]);
zk_sym = sym('z',[1 1]);
yk_sym = sym('y',[2 1]);

%% symbolische Funktionen Modell-Groß

% Momentengleichgewicht im Motor (Last als Eingangsgröße)
f1_sym = -(motor_Widerstand/motor_Induktivitaet) * x_sym(1) - (motor_BackEMFkoef/motor_Induktivitaet) * x_sym(2)  + (1/motor_Induktivitaet)* u_sym(1);
% Maschenregel in Motor
f2_sym = (motor_Drehmomentkoef/motor_traegheit) * x_sym(1)- (motor_Daempfung/motor_traegheit) * x_sym(2)  - (1/motor_traegheit)*u_sym(2);

%% symbolische Funktionen Modell-klein (Reibkraft wird vernachlässigt)

% Momentengleichgewicht im Motor1
f1k_sym = -(motor_Widerstand/motor_Induktivitaet)* xk_sym(1) - (motor_BackEMFkoef/motor_Induktivitaet) * xk_sym(2) + (1/motor_Induktivitaet)* uk_sym(1);
% Maschenregel in Motor1
f2k_sym = (1/(1-((2*Reifen_Radius*I_Bot)/(motor_traegheit*Achsabstand^2))-((m_ges*Reifen_Radius^2)/(4*motor_traegheit))))*((motor_Drehmomentkoef/motor_traegheit)*xk_sym(1)-(motor_BackEMFkoef/motor_Induktivitaet) * xk_sym(2)+ (((m_ges*Reifen_Radius^2)/(4*motor_traegheit)))-((2*Reifen_Radius*I_Bot)/(motor_traegheit*Achsabstand^2)))*xk_sym(4);
% Momentengleichgewicht im Motor2
f3k_sym = -(motor_Widerstand/motor_Induktivitaet)* xk_sym(3) - (motor_BackEMFkoef/motor_Induktivitaet) * xk_sym(4) + (1/motor_Induktivitaet)* uk_sym(2);
% Maschenregel in Motor2
f4k_sym = (1/(1+((2*Reifen_Radius*I_Bot)/(motor_traegheit*Achsabstand^2))-((m_ges*Reifen_Radius^2)/(4*motor_traegheit))))*((motor_Drehmomentkoef/motor_traegheit)*xk_sym(3)-(motor_BackEMFkoef/motor_Induktivitaet) * xk_sym(4)+ (((m_ges*Reifen_Radius^2)/(4*motor_traegheit)))+((2*Reifen_Radius*I_Bot)/(motor_traegheit*Achsabstand^2)))*xk_sym(2);

%% state functions
f_sym = [
    f1_sym;
    f2_sym;
    ];

%

fk_sym = [
    f1k_sym;
    f2k_sym;
    f3k_sym;
    f4k_sym;
    ];

%% output function
g_sym = [
    x_sym(2);
    ];

%

gk_sym = [
    xk_sym(2);
    xk_sym(4);
    ];
%% initial values
x0s = [
    0;
    0;
    ];

u0 = [
    0;
    0;
    ];

z0 = 0;

%

x0sk = [
    0;
    0;
    0;
    0;
    ];

u0k = [
    0;
    0;
    ];

z0k = 0;

%% system matrices (symbolic)

A_sym = jacobian(f_sym,x_sym);
b_sym = jacobian(f_sym,u_sym);
e_sym = jacobian(f_sym,z_sym);

c_sym = jacobian(g_sym,x_sym);

%

Ak_sym = jacobian(fk_sym,xk_sym);
bk_sym = jacobian(fk_sym,uk_sym);
ek_sym = jacobian(fk_sym,zk_sym);

ck_sym = jacobian(gk_sym,xk_sym);

%% system matrices (functions)

A_func = matlabFunction(A_sym,'Vars',{x_sym,u_sym,z_sym});
b_func = matlabFunction(b_sym,'Vars',{x_sym,u_sym,z_sym});
e_func = matlabFunction(e_sym,'Vars',{x_sym,u_sym,z_sym});

c_func = matlabFunction(c_sym,'Vars',{x_sym,u_sym,z_sym});

%

Ak_func = matlabFunction(Ak_sym,'Vars',{xk_sym,uk_sym,zk_sym});
bk_func = matlabFunction(bk_sym,'Vars',{xk_sym,uk_sym,zk_sym});
ek_func = matlabFunction(ek_sym,'Vars',{xk_sym,uk_sym,zk_sym});

ck_func = matlabFunction(ck_sym,'Vars',{xk_sym,uk_sym,zk_sym});

%% system matrices (numerical)

A = A_func(x0s,u0,z0);
b = b_func(x0s,u0,z0);
e = e_func(x0s,u0,z0);

c = c_func(x0s,u0,z0);
d = 0;

%

Ak = Ak_func(x0sk,u0k,z0k);
bk = bk_func(x0sk,u0k,z0k);
ek = ek_func(x0sk,u0k,z0k);

ck = ck_func(x0sk,u0k,z0k);
dk = 0;

%% state space system

sys = ss(A,b,c,d);

sysk = ss(Ak,bk,ck,dk);

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









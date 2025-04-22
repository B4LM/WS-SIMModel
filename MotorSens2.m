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
p_sym = sym('p',[6 1]);

%% symbolische Funktionen

% Momentengleichgewicht im Motor (Last als Eingangsgröße)
f1_sym = -(p_sym(1)/p_sym(2)) * x_sym(1) - (p_sym(3)/p_sym(2)) * x_sym(2)  + (1/p_sym(2))* u_sym(1);
% Maschenregel in Motor
f2_sym = (p_sym(4)/p_sym(5)) * x_sym(1)- (p_sym(6)/p_sym(5)) * x_sym(2)  - (1/p_sym(5))*u_sym(2);


%% state functions
f_sym = [
    f1_sym;
    f2_sym;
    ];

%% output function
g_sym = [
    x_sym(2);
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

p0 = [
    motor_Widerstand ;
    motor_Induktivitaet;
    motor_BackEMFkoef;
    motor_Drehmomentkoef;
    motor_traegheit;
    motor_Daempfung
    ];

%% system matrices (symbolic)

A_sym = jacobian(f_sym,x_sym);
b_sym = jacobian(f_sym,u_sym);
e_sym = jacobian(f_sym,z_sym);

c_sym = jacobian(g_sym,x_sym);

dbdp_sym = jacobian(b_sym,p_sym);
dAdp_sym = jacobian(A_sym,p_sym);

%% system matrices (functions)

A_func = matlabFunction(A_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});
b_func = matlabFunction(b_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});
e_func = matlabFunction(e_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});

c_func = matlabFunction(c_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});

dbdp_func = matlabFunction(dbdp_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});
dAdp_func = matlabFunction(dAdp_sym,'Vars',{x_sym,u_sym,p_sym,z_sym});


%% system matrices (numerical)

A = A_func(x0s,u0,p_sym,z0);
b = b_func(x0s,u0,p_sym,z0);
e = e_func(x0s,u0,p_sym,z0);

c = c_func(x0s,u0,p_sym,z0);
d = 0;

dbdp_0 = dbdp_func(x0s,u0,p_sym,z0);
dAdp_0 = dAdp_func(x0s,u0,p_sym,z0);

%% Parametersensitivität

S = inv(A)* (dbdp_0 - dAdp_0*x0s)

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
%show(myMap)









clc; clear; close all; 

%% general

g = 9.81;                    % [m/s^2]
m_ges = 1;                   % [kg]
m_last = 0.86;               % [kg]
Achsabstand = 0.15;          % [m]    
Motoruebersetzung = 100;     % [1]
B_dis = 0.25;                % [m]
R_dis = 0.15;                % [m]
Xi = 2/3;                    % [1]
I_Bot = 0.0072;              % [kg*m^2]
mue_g = 0.35;                % [1]

startPos = [0.3;0.3];
waypoint = [0.2;1.4];


%% Motormodell

motorinertia = 0.847 * 10^-7;  % [kg*m^2] 
motorinductivity = 0.0005;     % [H]
motorresistance =  3;          % [Ohm]
motordamping = 1.2 * 10^-6;    % [N*m*s] [0.005-> alt]
motortorquekoef = 0.0019;      % [N*m / A] [0.06533-> alt] [0.00174]
motorBackEMFkoef = 0.0019;     % [V*s / rad] [0.06533-> alt]

%% Reifenmodell

m_Reifen = 0.038;             % [kg]
mue = 0.002;                 % [1]
Reifen_Radius = 0.04;        % [m]

%% motor state space model

% A_alt = [0 1 0; 0 -motordamping/motorinertia motortorquekoef/motorinertia;0 -motorBackEMFkoef/motorinductivity -motorresistance/motorinductivity];
% B = [0; 0; 1/motorinductivity];
% CT = [0 1 0];
% D = [0];
% 
% x0 = [0; 0; 0];

%% second try state space model

% A22 = -motordamping/(motorinertia + (m_Reifen * Reifen_Radius^2)/50);
% A23 = motortorquekoef/(motorinertia + (m_Reifen * Reifen_Radius^2)/50);
% A32 = -motorBackEMFkoef/motorinductivity;
% A33 = -motorresistance/motorinductivity;
% 
% A_neu = [0 1 0; 0 A22 A23; 0 A32 A33];
% B_neu = [0; 0; 1/motorinductivity];
% E_neu = [0; -((m_last/2 + m_Reifen) / (motorinertia + (m_Reifen *Reifen_Radius^2)/50) * g * mue * Reifen_Radius); 0 ];
% CT_omega = [0 1 0];
% CT_phi = [1 0 0];
% CT_i = [0 0 1];
% x0_neu = [0; 0.01; 0];

%% third try state space model
% A22n = -motordamping/motorinertia;
% A23n = motortorquekoef/motorinertia;
% A_neu2 = [0 1 0; 0 A22n A23n; 0 A32 A33];
% E_neu2 = [0; 0; 0 ];

%% testing

%TestV = 12;

%omega = (1/(motordamping +((motortorquekoef^2)/motorresistance))) * ((motortorquekoef/motorresistance) * TestV - (m_last/2 + m_Reifen) * g *mue * Reifen_Radius)

% AA = [-(motordamping/motorinertia) motortorquekoef/motorinertia -(1/motorinertia); -motorBackEMFkoef/motorinductivity -motorresistance/motorinductivity 0; 0 0 0 ];
% BB = [0;1/motorinductivity;0];
% CCT = [0 0 1];
% xx0 = [ 0; 0; 0];

%% Sym-try
% input: V
% output: i -> T

%% Operating point

x0s = [
    0;
    0;
    ];

u0 = [0 ; 0];
z0 = 0;

%% Symbolic variables


x_sym = sym('x',[2 1]);
u_sym = sym('u',[2 1]);
z_sym = sym('z',[1 1]);
y_sym = sym('y',[1 1]);

%% Symbolic functions

% Momentengleichgewicht im Motor (Last als Eingangsgröße)
f1_sym = -(motorresistance/motorinductivity) * x_sym(1) - (motortorquekoef/motorinductivity) * x_sym(2)  + (1/motorinductivity)* u_sym(1);
% Maschenregel in Motor
f2_sym = (motortorquekoef/motorinertia) * x_sym(1)- (motordamping/motorinertia) * x_sym(2)  - (1/motorinertia)*u_sym(2);


%% state functions
f_sym = [
    f1_sym;
    f2_sym;
    ];

%% output function
g_sym = [
    x_sym(2);
    ];

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


%% Sys-Test

% Zeitvektor
t = 0:0.01:10; 

% Eingangsgröße (z.B. sinusförmig)
u = ones(length(t), 1) * [12, 0]; %3.466

% Anfangszustand (optional)
x0 = [0; 0];

% Simulation
[y, t, x] = lsim(sys, u, t, x0);

% Ergebnis plotten
plot(t, y* (0.6/pi)) 
xlabel('Zeit (s)')
ylabel('n_G [rpm]')
title('Systemantwort auf Eingangsgröße')
grid on

%% ruhelage
% U_ein = 12;
% T_ein = 0;
% nm = (motorresistance / (motortorquekoef^2 +  motorresistance * motordamping)) * ((motortorquekoef / motorresistance) * U_ein - T_ein)% * (60/(2*pi))/50;
% x1 = linspace(0,0.1,1000); 
% nmt1 = (motorresistance ./ (x1.^2 +  motorresistance * motordamping)) .* ((x1 / motorresistance) * U_ein - T_ein) * (60/(2*pi))* (1/Motoruebersetzung);
% 
% plot(x1, nmt1)
% xlim([0 0.1])

%%%%%

% syms x1
% f = (motorresistance / (x1^2 +  motorresistance * motordamping)) * ((x1 / motorresistance) * U_ein - T_ein) * (60/(2*pi))/100;
% f2 = diff(f,x1)==0;
% extreme_points = solve(f2,x1);
% extreme_values = subs(f, x1, extreme_points);
% [maxX, maxidx] = max(extreme_values);
% best_location = extreme_points(maxidx);
% best_value = simplify(maxX, 'steps', 50);

%% Map

myMap = binaryOccupancyMap(1.8,1.8,100);

walls = zeros(180,180);
walls(15,15:165) = 1; % Top wall
walls(165,15:165) = 1; % Bottom wall
walls(15:165,15) = 1; % Left wall
walls(15:165,165) = 1; % Right wall
walls((180-waypoint(2)*100)-1:(180-waypoint(2)*100)+1,(waypoint(1)*100)-1:(waypoint(1)*100)+1) = 1; %waypoint


setOccupancy(myMap,[1 1], walls, "grid")
show(myMap)
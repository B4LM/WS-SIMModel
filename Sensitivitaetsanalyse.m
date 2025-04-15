clc; clear; close all; 

%% Wegpunkte

StartPos = [0.3;0.3];     

%% Parameter

%Allgemein
g = 9.81;                           % [m/s^2]
m_ges = 1;                          % [kg]
Achsabstand = 0.15;                 % [m]    
Motoruebersetzung = 100;            % [1]
B_dis = 0.25;                       % [m]
R_dis = 0.15;                       % [m]
Xi = 2/3;                           % [1]
I_Bot = 0.0072;                     % [kg*m^2]
mue_g = 0.1;                        % [1]

%Motor
motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s][7.216 * 10^-4]
motor_Drehmomentkoef = 0.0174;      % [N*m / A]
motor_BackEMFkoef = 0.0035;         % [V*s / rad]

%Reifen
m_Reifen = 0.038;                   % [kg]               
Reifen_Radius = 0.04;               % [m]

%; B_dis;R_dis ;Xi;I_Bot;mue_g;motor_traegheit;motor_Induktivitaet;motor_Widerstand;motor_Daempfung;motor_Drehmomentkoef;motor_BackEMFkoef;m_Reifen,Reifen_Radius
Params= [m_ges; Achsabstand;Motoruebersetzung];
Params_0 = Params;

for i = 1:length(Params)
    for j=1:5
        i
        j
        switch j
            case 1
                Params(i)= 0.9*Params(i);
            case 2
                Params(i)= 0.95*Params(i);
            case 4
                Params(i)= 1.05*Params(i);
            case 5
                Params(i)= 1.1*Params(i);
        end
    %Simulationsvorbereitung
    x_sym = sym('x',[2 1]);
    u_sym = sym('u',[2 1]);
    z_sym = sym('z',[1 1]);
    y_sym = sym('y',[1 1]);

    f1_sym = -(motor_Widerstand/motor_Induktivitaet) * x_sym(1) - (motor_BackEMFkoef/motor_Induktivitaet) * x_sym(2)  + (1/motor_Induktivitaet)* u_sym(1);
    f2_sym = (motor_Drehmomentkoef/motor_traegheit) * x_sym(1)- (motor_Daempfung/motor_traegheit) * x_sym(2)  - (1/motor_traegheit)*u_sym(2);
    
    f_sym = [
    f1_sym;
    f2_sym;
    ];

    g_sym = [
    x_sym(2);
    ];

    x0s = [
        0;
        0;
        ];
    
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
    
    ki= Achsabstand
    %Simulation selber

    %Reset
    Params(i)=Params_0(i);
    end
end

% Die Eingänge müssen im Simulink Modell eingesetllt werden!

clc; clear; close all; 

%% Wegpunkte

StartPos = [0.3;0.3];
Zielpunkt = [1;1.2]; %benötigt PID-Regler

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

%; 'B_dis','R_dis'
%,'Xi','I_Bot','mue_g','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef','m_Reifen','Reifen_Radius',
%'Achsabstand','Motoruebersetzung'
ParamsList= {'m_ges','Achsabstand','B_dis','R_dis','Xi','I_Bot','mue_g','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef','m_Reifen','Reifen_Radius','Motoruebersetzung'};


pstruct = struct();

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

    %Simulation selber
    load_system("SimpleBot_V3.slx")
    set_param("SimpleBot_V3/Robot Visualizer","Commented","on")
    disp('Sim startet!')
    out = sim('SimpleBot_V3');
    disp('Sim beendet!')

    %Results

    pstruct.(ParamsList{i}).savePosData(out.position.Data,j);
    pstruct.(ParamsList{i}).saveVelData(out.velocity.Data,j);
    pstruct.(ParamsList{i}).saveTime(out.velocity.Time,j);
    %pstruct.(ParamsList{i}).position(j) = [out.position.Data(:,1), out.position.Data(:,2)];
    %pstruct.(ParamsList{i}).velocity(j) = [out.velocity.Time, out.velocity.Data];

    end
    assignin('base',ParamsList{i},pstruct.(ParamsList{i}).value)
end

for pn = 1: length(ParamsList)
    plotSensibilities(pstruct,ParamsList(pn))
    input('nächter Parameter');
end




% Die Eingänge müssen im Simulink Modell eingesetllt werden!
clc; clear; close all; 

%% Allgemeine Parameter
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

%% Motorparameter
motor_traegheit = 0.847 * 10^-6;    % [kg*m^2] 
motor_Induktivitaet = 0.0002;       % [H]
motor_Widerstand =  13.33;          % [Ohm]
motor_Daempfung = 9.12 * 10^-8;     % [N*m*s]
motor_Drehmomentkoef = 0.0035;      % [N*m / A]              
motor_BackEMFkoef = 0.0035;         % [V*s / rad]



%% Sim Parameters
t_span = [0 1.5];            % [s]
StartPos = [0.3;0.3];        % [m]
Zielpunkt = [1;1.2];         % [m]
U_sim = [10.5; 12];          % [V]

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

%% Liste der Parameter, kann auf interessierte reduziert werden
%,'Achsabstand','Motoruebersetzung','B_dis','Xi','I_Bot','mue_g','Reifen_Radius','I_Reifen','L_B','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef'
ParamsList= {'g','m_ges','Achsabstand','Motoruebersetzung','B_dis','Xi','I_Bot','mue_g','Reifen_Radius','I_Reifen','L_B','motor_traegheit','motor_Induktivitaet','motor_Widerstand','motor_Daempfung','motor_Drehmomentkoef','motor_BackEMFkoef'};
p_list_latex = {'g', '$m_{ges}$','Achsabstand', '$i_G$', '$B_{dis}$', '$\xi$', '$I_{Bot}$', '$\mu_g$','r', '$I_{Reifen}$', '$L_B$', '$I_M$', '$L_M$', '$\Omega_M$','$b_M$', '$k_T$','$k_B$'};

pstruct = struct();

for n=1:length(ParamsList)
    pname = ParamsList{n};
    pstruct.(pname) = Sensibilities(eval(pname));
end


for i = 1:length(ParamsList)
    disp(ParamsList{i})
    for j=1:5 % ändern der Parameter im Workspace
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
        
    %% Simulation selber
    load_system("BotModel2Sens.slx")
    disp('Sim startet!')
    out = sim('BotModel2Sens.slx');
    disp('Sim beendet!')

    %Ergebnisse
    pstruct.(ParamsList{i}).savePosData(out.position.Data,j);
    pstruct.(ParamsList{i}).saveVelData(out.velocity.Data,j);
    pstruct.(ParamsList{i}).saveTime(out.velocity.Time,j);

    end
    assignin('base',ParamsList{i},pstruct.(ParamsList{i}).value)
end

%% plotten
for pn = 1: length(ParamsList)
    plotSensibilities(pstruct,ParamsList{pn},p_list_latex{pn})
    input('nächter Parameter');
end




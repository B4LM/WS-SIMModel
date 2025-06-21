clc; clear; close all; 

motorinertia = 0.847 * 10^-6;        % [kg*m^2] 
motorinductivity = 0.0002;       % [H]
motorresistance =  13.33;            % [Ohm]
motordamping = 9.12 * 10^-8;          % [N*m*s][7.216 * 10^-4];
motortorquekoef = 0.0035;      % [N*m / A]
motorBackEMFkoef = 0.0035;     % [V*s / rad]
%V_brush = 1;                         % [V]

U_e = 12;

%om = (motortorquekoef / (motorresistance*motordamping + motortorquekoef*motorBackEMFkoef)) * U_e

T_ein = 0;
om = (motorresistance / (motortorquekoef*motorBackEMFkoef +  motorresistance * motordamping)) * ((motortorquekoef / motorresistance) * U_e - T_ein);
rpm = (om*60)/ (2*pi*100)


Tf = linspace(0,0.05,100);
omf = (motorresistance / (motortorquekoef*motorBackEMFkoef +  motorresistance * motordamping)) * ((motortorquekoef / motorresistance) * U_e - Tf);
syms x
eqn = (motorresistance / (motortorquekoef*motorBackEMFkoef +  motorresistance * motordamping)) * ((motortorquekoef / motorresistance) * U_e-x) == 0;
plot(Tf, omf)
xlim([0 0.05])

Ts = solve(eqn,x)

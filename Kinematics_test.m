omega1 = 2 ;
omega2 = 7;
omegadot1 = 3;
omegadot2 = 1;
phi = 30;
rs = [0;0];
omegabotdot = 2;

RotPhi = [cosd(phi) -sind(phi); sind(phi) cosd(phi)];

r10 = rs + RotPhi * [B_dis/3 ; Achsabstand/2];
r20 = rs + RotPhi * [B_dis/3 ; -Achsabstand/2];

v10 = RotPhi * [1;0] * omega1 * Reifen_Radius;
v20 = RotPhi * [1;0] * omega2 * Reifen_Radius;
vs = RotPhi * [1;0] * ((omega1 * Reifen_Radius + omega2 * Reifen_Radius)/2);

o_numerator = (v20(1) - v10(1)) * (r10(2) - r20(2)) + (v20(2) - v10(2)) * (r20(1) - r10(1));
o_denominator = (r10(2) - r20(2))^2 + (r20(1) - r10(1))^2;
omega_Bot = o_numerator / o_denominator;

a10 = RotPhi * [1;0] * omegadot1 * Reifen_Radius;
a20 = RotPhi * [1;0] * omegadot2 * Reifen_Radius;
as = RotPhi * [1;0] * ((omegadot1 * Reifen_Radius + omegadot2 * Reifen_Radius)/2);

odot_numerator = (a20(1) - a10(1) - omega_Bot^2 * (r10(1) - r20(1))) * (r10(2) - r20(2)) + (a20(2) - a10(2) - omega_Bot^2 * (r10(2) - r20(2))) * (r20(1) - r10(1));
odot_denominator = (r10(2) - r20(2))^2 + (r20(1) - r10(1))^2;
omegadot_Bot = odot_numerator/odot_denominator;

%Formeln in Notizen übersichtlicher!

%Berechnung-Kraftrichtung-Block-Reibkräfte-> Alpha
%Geschwindigkeitspol xp
xp = (Achsabstand/2) * ((omega1 *Reifen_Radius + omega2 * Reifen_Radius)/(omega2 * Reifen_Radius-omega1 *Reifen_Radius));
%Berechnung-Winkel
alpha = atan(B_dis / xp);

%Brechnung-Boden-Kontakt-Kräfte

Fg_Bot = m_ges * g;                 %Schwerpunkt bekannt
%F_Reifen = Fg_Bot * ((1-Xi)/2);
F_Block = Fg_Bot * (Xi/2);

%Brechung der Lastmomente T1 & T2

T1 = -(Reifen_Radius/2)* (m_ges * as(1) - (2/Achsabstand)* I_Bot * omegabotdot - F_Block * (2*cos(alpha) + (4/Achsabstand)*B_dis*sin(alpha)*(1-Xi)));

T2 = -(Reifen_Radius/2)* (m_ges * as(1) + (2/Achsabstand)* I_Bot * omegabotdot - F_Block * (2*cos(alpha) - (4/Achsabstand)*B_dis*sin(alpha)*(1-Xi)));










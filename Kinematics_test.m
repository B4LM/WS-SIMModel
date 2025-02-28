omega1 = 90 ;
omega2 = 1550;
omegadot1 = 0;
omegadot2 = 0;
phi = 0;
rs = [0;0];
omega_GB1 = Motoruebersetzung * omega1;
omega_GB2 = Motoruebersetzung * omega2;

RotPhi = [cosd(phi) -sind(phi); sind(phi) cosd(phi)];

r10 = rs + RotPhi * [-B_dis*Xi; Achsabstand/2];
r20 = rs + RotPhi * [-B_dis*Xi ; -Achsabstand/2];

v10 = RotPhi * [1;0] * omega_GB1 * Reifen_Radius;
v20 = RotPhi * [1;0] * omega_GB2 * Reifen_Radius;
vs = RotPhi * [1;0] * ((omega_GB1 * Reifen_Radius + omega_GB2 * Reifen_Radius)/2);

o_numerator = (v20(1) - v10(1)) * (r10(2) - r20(2)) + (v20(2) - v10(2)) * (r20(1) - r10(1));
o_denominator = (r10(2) - r20(2))^2 + (r20(1) - r10(1))^2;
omega_Bot = o_numerator / o_denominator;

a10 = RotPhi * [1;0] * omegadot1 * Reifen_Radius;
a20 = RotPhi * [1;0] * omegadot2 * Reifen_Radius;
as = RotPhi * [1;0] * ((omegadot1 * Reifen_Radius + omegadot2 * Reifen_Radius)/2);
as_bot = RotPhi' * as; 

odot_numerator = (a20(1) - a10(1) - omega_Bot^2 * (r10(1) - r20(1))) * (r10(2) - r20(2)) + (a20(2) - a10(2) - omega_Bot^2 * (r10(2) - r20(2))) * (r20(1) - r10(1));
odot_denominator = (r10(2) - r20(2))^2 + (r20(1) - r10(1))^2;
omegadot_Bot = odot_numerator/odot_denominator;









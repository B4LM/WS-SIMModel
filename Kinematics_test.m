omega1 = 2 ;
omega2 = 1;
omegadot1 = 1;
omegadot2 = 1;
phi = 30;
rs = [0;0];

RotPhi = [cosd(phi) -sind(phi); sind(phi) cosd(phi)];

r10 = rs + RotPhi * [B_dis/3 ; Achsabstand/2];
r20 = rs + RotPhi * [B_dis/3 ; -Achsabstand/2];

v10 = RotPhi * [1;0] * omega1 * Reifen_Radius;
v20 = RotPhi * [1;0] * omega2 * Reifen_Radius;

Avel = [1 0 r10(1) ; 0 1 r10(2); 1 0 r20(1); 0 1 r20(2)];
Vel_Result = linsolve(Avel,[v10;v20])

%a1 = phivec * omegadot1 * Reifen_Radius;
%a2 = phivec * omegadot2 * Reifen_Radius;
r_neu = sqrt(B_dis^2 +(Achsabstand/2)^2);
v1_neu = omega1 * Reifen_Radius;
v2_neu = omega2 * Reifen_Radius;
Vel_Result_neu = linsolve([1 r_neu; 1 r_neu], [v1_neu; v2_neu])




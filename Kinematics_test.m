omega1 = 10;
omega2 = 1;
omegadot1 = 1;
omegadot2 = 1;
phi = 5;
%phi = phideg * 180/pi;

phivec = [cosd(phi); sind(phi)];

%r1x = (1/3)*cos(phi)* B_dis - (1/2)*sin(phi)*Achsabstand;
%r1y = (1/3)*sin(phi)* B_dis + (1/2)*cos(phi)*Achsabstand;
r1 = [(1/3)*phivec(1)* B_dis - (1/2)*phivec(2)*Achsabstand;(1/3)*phivec(2)* B_dis + (1/2)*phivec(1)*Achsabstand];
%r2x = (1/3)*cos(phi)* B_dis + (1/2)*sin(phi)*Achsabstand;
%r2y = (1/3)*sin(phi)* B_dis - (1/2)*cos(phi)*Achsabstand;
r2 = [(1/3)*phivec(1)* B_dis + (1/2)*phivec(2)*Achsabstand;(1/3)*phivec(2)* B_dis - (1/2)*phivec(1)*Achsabstand];

v1 = phivec * omega1 * Reifen_Radius;
v2 = phivec * omega2 * Reifen_Radius;

Avel = [1 0 r1(1) ; 0 1 r1(2); 1 0 r2(1); 0 1 r2(2)];
Vel_Result = linsolve(Avel,[v1;v2])

a1 = phivec * omegadot1 * Reifen_Radius;
a2 = phivec * omegadot2 * Reifen_Radius;



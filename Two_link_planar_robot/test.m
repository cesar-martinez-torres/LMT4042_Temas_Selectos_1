syms th1 th2 L1 L2
R1 = rotMatrix(th1,'deg');     % 30° rotation
T1 = translMatrix([L1 0]);     % Translation of (5,2)
H1 = R1 * T1;                   % Full homogeneous transformation
R2 = rotMatrix(th2,'deg');     % 30° rotation
T2 = translMatrix([L2 0]);     % Translation of (5,2)
H2 = R2 * T2;                   % Full homogeneous transformation
H= H1*H2
th1=55
th2=35
L1=1.2
L2=1.2
Ht=eval(H)
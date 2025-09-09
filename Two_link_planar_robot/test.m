syms th1 th2 L1 L2
R = rotMatrix(th1);     % 30° rotation
T = translMatrix([L1 0]);     % Translation of (5,2)
H = T * R;                   % Full homogeneous transformation

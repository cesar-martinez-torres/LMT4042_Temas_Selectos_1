%%PROGRAMA PARA GRAFICAR LOS RESULTADOS DE LAS SIMULACIONES DEL ARCHIVO
%%SIMLULINK SIMU1, DEL ARTICULO DEL RIAI
close all
figure(1)
subplot(2,2,1)
plot(t,y)
xlabel('Tiempo')
ylabel('y')
title('Movimiento en el eje y')
grid on

subplot(2,2,2)
plot(t,yp)
xlabel('Tiempo')
ylabel('dy/dt')
title('Derivada de y con respecto al tiempo')
grid on

subplot(2,2,3)
plot(t,phi)
xlabel('Tiempo')
ylabel('phi')
title('Roll')
grid on

subplot(2,2,4)
plot(t,phip)
xlabel('Tiempo')
ylabel('dphi/dt')
title('Derivada de phi con respecto al tiempo')
grid on

%--------------------------------------------------------------------------
figure(2)
subplot(2,2,1)
plot(t,x)
xlabel('Tiempo')
ylabel('x')
title('Movimiento en el eje x')
grid on

subplot(2,2,2)
plot(t,xp)
xlabel('Tiempo')
ylabel('dx/dt')
title('Derivada de x con respecto al tiempo')
grid on

subplot(2,2,3)
plot(t,th)
xlabel('Tiempo')
ylabel('theta')
title('Yaw')
grid on

subplot(2,2,4)
plot(t,phip)
xlabel('Tiempo')
ylabel('dtheta/dt')
title('Derivada de theta con respecto al tiempo')
grid on
%--------------------------------------------------------------------------
figure(3)
subplot(2,2,1)
plot(t,z)
xlabel('Tiempo')
ylabel('z')
title('Movimiento en el eje z')
grid on

subplot(2,2,2)
plot(t,zp)
xlabel('Tiempo')
ylabel('dz/dt')
title('Derivada de z con respecto al tiempo')
grid on

subplot(2,2,3)
plot(t,psi)
xlabel('Tiempo')
ylabel('psi')
title('pitch')
grid on

subplot(2,2,4)
plot(t,psip)
xlabel('Tiempo')
ylabel('dpsi/dt')
title('Derivada de psi con respecto al tiempo')
grid on

%--------------------------------------------------------------------------
figure(4)
subplot(2,2,1)
plot(t,u)
xlabel('Tiempo')
ylabel('Control de altura')
title('Control suma de fuerzas 4 motores')
grid on

subplot(2,2,2)
plot(t,tpsi)
xlabel('Tiempo')
ylabel('par en psi')
title('par Yaw')
grid on

subplot(2,2,3)
plot(t,tth)
xlabel('Tiempo')
ylabel('par en theta')
title('par  Pitch')
grid on

subplot(2,2,4)
plot(t,tphi)
xlabel('Tiempo')
ylabel('par en phi')
title('par Roll')
grid on




























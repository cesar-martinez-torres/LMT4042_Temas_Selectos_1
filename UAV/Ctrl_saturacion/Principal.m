%% Programa para simular los controles del articulo Modelado y
%% estabilizacion de un helicoptero de 4 rotores de P. Castillo et al. de
%% la RIAI.
clear all
close all
%% Parametros de control
az1=3;
az2=0.5;
apsi1=2.374;
apsi2=0.08;
bphi1=2;
bphi2=1;
bphi3=0.2;
bphi4=0.1;
bth1=2;
bth2=1;
bth3=0.2;
bth4=0.1;
T=17;
m=0.52;
zd=50;
psid=0.5;
g=9.81;
mg=m*g;
%% Condiciones iniciales de los integradores.
ci_zp=0;
ci_z=0;
ci_psip=0;
ci_psi=0;

ci_yp=0;
ci_y=70;
ci_phip=0;
ci_phi=0;

ci_xp=0;
ci_x=100;
ci_thp=0;
ci_th=0;
%% Simular y ejecutar el controlador y el graficador
sim ('nested.slx') % Archivo simulink con el controlador de saturaciones anidadas
run graficador
%%






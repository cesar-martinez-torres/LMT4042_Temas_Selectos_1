tf=150 % Number of Path points
A=[tf^2 tf^3; 2*tf 3*tf^2]%Matrix A
%% For "th1" 
xd=10 % Desired final position in th1 
xpd=0 % Desired final velocity in th1
a0=0  % Initial condition in th1 position
a1=0  % Initial condition in th1 velocity
B=[10;0]
coef=(inv(A))*B
a2=coef(1)
a3=coef(2)
rev=A*coef

for n=1:1:tf
    %%  Calculation of all path points
    px(n)=a0+a1*n+a2*n^2+a3*n^3; % Position in th1
    vx(n)=a1+2*a2*n+3*a3*n^2; % Velocity in th1
    t(n)=n;
    
end

figure('Name','Trajectories in th1','NumberTitle','off')
subplot(2,1,1)
plot(t,px)
axis([0 tf+3 0 xd+3]) 
xlabel('Time [s]','FontSize',10)
ylabel('')
title('Position','FontSize',10)
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,2)
plot(t,vx)
axis([0 tf+3 0 0.12]) 
xlabel('Time [s]','FontSize',10)
ylabel('')
title('Velocity','FontSize',10)
grid on

clc; clear; close all;
dt = 0.01;
tf = 30;
tspan = 0:dt:tf-dt;
wr = 0;
vr = 0.5;
lyapunovGain = [0.5, 0.2, 0.6]; %model 1
% lyapunovGain = [1 0.01 3];
% lyapunovGain = [200 500 100]; %model 2
x_init = [1,1,deg2rad(0)];
[t,x] = ode45(@(t,x)errordynamics(t,x,vr,wr,lyapunovGain),tspan,x_init);
figure,plot(tspan,x(:,1),tspan,x(:,2),tspan,x(:,3)),grid on
xlabel('time(sec)'),ylabel('amplitude'),legend('\it x_e','\it y_e','\it\Theta_e')
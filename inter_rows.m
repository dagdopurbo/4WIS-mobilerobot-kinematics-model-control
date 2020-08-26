clc; clear; close all;

%% time parameter
dt = 0.1;
t0 = 0;
tf = 60-dt;
tspan = (t0:dt:tf)';

%% Define the Robot parameters
Lf = 0.235; % m
Lr = 0.235; % m
wheel_rad = 0.12; % m
W = 0.42; % m
m = 20; % Kg

%% Define way points
% numWaypoints = (0:0.5:10)';

% xRef = [zeros(length(numWaypoints)-1,1); numWaypoints(1:end-1); 10*ones(length(numWaypoints),1);...
%         1*flipud(numWaypoints)];
%     
% yRef = [numWaypoints(1:end-1); 10*ones(length(numWaypoints)-1,1); 1*flipud(numWaypoints);...
%         zeros(length(numWaypoints),1)];
% 
% thetaRef = [deg2rad(90)*ones(length(numWaypoints)-1,1); deg2rad(0)*ones(length(numWaypoints)-1,1);...
%             deg2rad(-90)*ones(length(numWaypoints)-1,1); deg2rad(-180)*ones(length(numWaypoints)+1,1)];

numWaypoints = (0:0.25:5)';

xRef = [zeros(length(numWaypoints),1); 1*ones(length(numWaypoints),1);  2*ones(length(numWaypoints),1);...
        3*ones(length(numWaypoints),1); 4*ones(length(numWaypoints),1); 5*ones(length(numWaypoints),1)];

yRef = [numWaypoints; 1*flipud(numWaypoints); numWaypoints; 1*flipud(numWaypoints); numWaypoints; 1*flipud(numWaypoints)];

thetaRef = [deg2rad(90)*ones(length(numWaypoints)-1,1); deg2rad(65); deg2rad(35);...
            deg2rad(-90)*ones(length(numWaypoints)-2,1); deg2rad(-65); deg2rad(-35);...
            deg2rad(90)*ones(length(numWaypoints)-2,1); deg2rad(65); deg2rad(35);...
            deg2rad(-90)*ones(length(numWaypoints)-2,1); deg2rad(-65); deg2rad(-35);...
            deg2rad(90)*ones(length(numWaypoints)-2,1); deg2rad(0); deg2rad(-10);...
            deg2rad(-90)*ones(length(numWaypoints)-1,1);];
            
%% Reference Path        
Reference = [xRef,yRef,thetaRef];

%% Initial Position
% initialPosition = [0.5;0;deg2rad(180)];
initialPosition = [-1;-1;deg2rad(0)];

xAct(1,:) = initialPosition(1);
yAct(1,:) = initialPosition(2);
thetaAct(1,:) = initialPosition(3);

vwMax = [0.5,1]; % maximum velocity, maximum omega
vwDes = [0.5,0]; % desired velocity, desired omega

%% Controller Gain
lyapunovGain = [0.5, 0.2, 1]; % k1, k2, k3
% lyapunovGain = [1 0.01 3]; % Model 1
% lyapunovGain = [200 500 100]; % Model 2
%% index toko indonesia
k = 1;
waypointIndex = 0;
last_waypointIndex = [];
numsinTolerance = 1e-2; % Numerical Singularity Tolerance
cycle = 1;

viz = Visualizer2D;
while tspan(k,:) < tf
%% Waypoints initializing
      
    if waypointIndex == 0
        waypointIndex = 1;
    elseif waypointIndex == length(xRef)+1
        waypointIndex = 1;
        cycle = cycle + 1;
    else
        % do nothing
    end
    
    last_waypointIndex(k,:) = waypointIndex;

%% Kinematics Control
    [vRef(k,:),wRef(k,:),waypointIndex,xe(k,:),ye(k,:),thetae(k,:)] = kinematics_control(lyapunovGain,[xAct(k,:),yAct(k,:),thetaAct(k,:)],Reference,vwMax,vwDes,waypointIndex);

% Numerical Singularity Solver in Speed Controller
    if abs(vRef(k,:)) < numsinTolerance
        waypointIndex = waypointIndex + ceil(length(numWaypoints)/10)+2;
    else
        % do nothing
    end
    
%% Inverse Kinematics
    [delta(:,k), wheelspeed(:,k)] = ugv_inverseKinematics(vRef(k,:),wRef(k,:),Lf,Lr,wheel_rad);
%% Forward Kinematics
    [vxAct(k,:),omegaAct(k,:)] = ugv_forwardKinematics(delta(:,k),wheelspeed(:,k),Lf,Lr,W);
    
%% Conversion of local to global frame
    xdotAct(k,:) = vxAct(k,:)*cos(thetaAct(k,:));% - vyAct*sin(thetaAct(k,:));
    ydotAct(k,:) = vxAct(k,:)*sin(thetaAct(k,:));% + vyAct*cos(thetaAct(k,:));
    thetadotAct(k,:) = omegaAct(k,:);
 
%% Get Tractor Position in global frame by integration
    xAct(k+1,:) = xAct(k,:) + xdotAct(k,:)*dt;
    yAct(k+1,:) = yAct(k,:) + ydotAct(k,:)*dt;
    thetaAct(k+1,:) = thetaAct(k,:) + thetadotAct(k,:)*dt;
    if thetaAct(k+1,:) < 2*pi
       % do nothing
    else
       thetaAct(k+1,:) = thetaAct(k+1,:) - 2*pi;
    end    
    
%% Run the motion animation     


    delta1(k,:) = delta(1,k);
    delta2(k,:) = delta(2,k);
    delta3(k,:) = delta(3,k);
    delta4(k,:) = delta(4,k);
    velocity1(k,:) = wheelspeed(1,k);
    velocity2(k,:) = wheelspeed(2,k);
    velocity3(k,:) = wheelspeed(3,k);
    velocity4(k,:) = wheelspeed(4,k);
    
    refX(k,:) = xRef(waypointIndex);
    refY(k,:) = yRef(waypointIndex);
    refTheta(k,:) = thetaRef(waypointIndex);
    k = k+1;
end

n = 10;
for i = 1:1:length(xAct)/n
    xpose(i,:) = xAct(i*n,:);
    ypose(i,:) = yAct(i*n,:);
    thetapose(i,:) = thetaAct(i*n,:);
end

xpose = [xAct(1,:); xpose];
ypose = [yAct(1,:); ypose];
thetapose = [thetaAct(1,:); thetapose];

figure,
plot(xRef(1:21),yRef(1:21),'r--'),hold on,
plot(xRef(22:42),yRef(22:42),'r--'),hold on,
plot(xRef(43:63),yRef(43:63),'r--'),hold on,
plot(xRef(64:84),yRef(64:84),'r--'),hold on,
plot(xRef(85:105),yRef(85:105),'r--'),hold on,
plot(xRef(106:126),yRef(106:126),'r--'),hold on,
plot(xAct,yAct,'-b'),hold on,grid on
for i = 1:1:length(xpose)
    xyzpose = [xpose(i,:) ypose(i,:) 0];
    rotpose = axang2quat([0 0 1 thetapose(i,:)]);
%     plotTransforms(xyzpose, rotpose,...
%         'MeshFilePath', 'groundvehicle.stl', "View", "2D",...
%         'MeshColor', 'blue','FrameSize',0.5);
    ugv_shape(Lf+Lr,W,xpose(i,:),ypose(i,:),rad2deg(thetapose(i,:)),'k',0.5),
    hold on
end
light;
xlabel('X (m)'), ylabel('Y (m)'),axis ([-2 6 -2 6])

%% Figure plotting
figure,
plot(xRef(1:21),yRef(1:21),'r--'),hold on,
plot(xRef(22:42),yRef(22:42),'r--'),hold on,
plot(xRef(43:63),yRef(43:63),'r--'),hold on,
plot(xRef(64:84),yRef(64:84),'r--'),hold on,
plot(xRef(85:105),yRef(85:105),'r--'),hold on,
plot(xRef(106:126),yRef(106:126),'r--'),hold on,
plot(xAct,yAct,'-b'), grid on, legend('reference','actual')
xlabel('X (m)'), ylabel('Y (m)'),axis ([-2 6 -2 6])

figure,subplot(2,1,1),plot(tspan(1:end),0.5*ones(length(vxAct)+1,1),tspan(1:end),[0;vxAct]),grid on
xlabel('time(sec)'), ylabel('velocity(m/s)'), legend('reference','actual'),axis([0 tf 0 0.6])

subplot(2,1,2),plot(tspan(1:end),zeros(length(omegaAct)+1,1),tspan(1:end),[0;omegaAct]),grid on
xlabel('time(sec)'), ylabel('omega(rad/s)'), legend('reference','actual'),axis([0 tf -1.3 1.3])


figure, subplot(2,1,1), plot(tspan(1:end-1),rad2deg(delta1),tspan(1:end-1),rad2deg(delta4)),grid on
xlabel('time(sec)'), ylabel('angle(degree)'), legend('front left (\delta_1)','back left (\delta_4)')
subplot(2,1,2), plot(tspan(1:end-1),rad2deg(delta2),tspan(1:end-1),rad2deg(delta3)),grid on
xlabel('time(sec)'), ylabel('angle(degree)'), legend('front right (\delta_2)','back right (\delta_3)')

figure, subplot(2,1,1), plot(tspan(1:end-1),velocity1,tspan(1:end-1),velocity4),grid on
xlabel('time(sec)'), ylabel('wheel velocity(m/s)'), legend('front left (v_1)','back left (v_4)')
subplot(2,1,2), plot(tspan(1:end-1),velocity2,tspan(1:end-1),velocity3),grid on
xlabel('time(sec)'), ylabel('wheel velocity(m/s)'), legend('front right (v_2)','back right (v_3)')

% figure, subplot(2,1,1), plot(tspan(1:end-1),velocity1,tspan(1:end-1),velocity4),grid on
% xlabel('time(sec)'), ylabel('angle(degree)'), legend('front left','back left')
% subplot(2,1,2), plot(tspan(1:end-1),velocity2,tspan(1:end-1),velocity3),grid on
% xlabel('time(sec)'), ylabel('angle(degree)'), legend('front right','back right')

figure,plot(refX),hold on,plot(xAct)
figure,plot(refY),hold on,plot(yAct)
figure,plot(refTheta),hold on,plot(thetaAct)

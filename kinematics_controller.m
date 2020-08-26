function [v,w,i,xn,ye,thetae] = kinematics_controller(controllerGain,actualPose,referencePose,maxSpeed,desSpeed,i)
% NOTE:
% This is a Lyapunov based nonlinear kinematics controller
  
    
% For Numerical Singularity tolerancs
    numsingTolerance = 1e-3;

% Maximum velocity and omega
    maximumVelocity = maxSpeed(1);
    maximumOmega = maxSpeed(2);
    
% Desired velocity and omega
    desiredVelocity = desSpeed(1);
    desiredOmega = desSpeed(2);

% Extracting x y theta
    xAct = actualPose(1);
    yAct = actualPose(2);
    thetaAct = actualPose(3);
    
    xRef = referencePose(:,1);
    yRef = referencePose(:,2);
    thetaRef = referencePose(:,3);

% Controller Gain
    k1 = controllerGain(1);
    k2 = controllerGain(2);
    k3 = controllerGain(3);

    if i == length(xRef)
        i = 1;
    else
        % do nothing
    end
    
% Waypoints setup
    if i == 1
        xn = (xAct - xRef(i,:))*cos(thetaRef(i,:)) + (yAct - yRef(i,:))*sin(thetaRef(i,:));
    elseif i == length(xRef)
        xn = (xAct - xRef(i,:))*cos(thetaRef(i,:)) + (yAct - yRef(i,:))*sin(thetaRef(i,:));
    else
        xn = (xAct - xRef(i,:))*cos((thetaRef(i,:)+thetaRef(i-1,:))/2) + (yAct - yRef(i,:))*sin((thetaRef(i,:)+thetaRef(i-1,:))/2);
    end
    
% Error Calculation
    if xn < 0
        xe = (xRef(i,:) - xAct)*cos(thetaAct) + (yRef(i,:) - yAct)*sin(thetaAct);
        ye = -(xRef(i,:) - xAct)*sin(thetaAct) + (yRef(i,:) - yAct)*cos(thetaAct);
        thetae = thetaRef(i,:) - thetaAct;
    else
        if i < length(xRef)
            i = i + 1;
        else
            i = length(xRef);
        end
        xe = (xRef(i,:) - xAct)*cos(thetaAct) + (yRef(i,:) - yAct)*sin(thetaAct);
        ye = -(xRef(i,:) - xAct)*sin(thetaAct) + (yRef(i,:) - yAct)*cos(thetaAct);
        thetae = thetaRef(i,:) - thetaAct;
    end
    
        
% Lyapunov Controller Output
%     v = k1*xe + desiredVelocity*cos(thetae);
    v = k1*xe + desiredVelocity*cos(thetae);
    if v > maximumVelocity
        v = maximumVelocity;
    elseif v < -maximumVelocity
        v = -maximumVelocity;
    else
        % do nothing
    end
    
    w = (1/k2)*ye*desiredVelocity + k3*sin(thetae) + desiredOmega;
%     if thetae < numsingTolerance
%         w = k2*ye*desiredVelocity + k3*sin(thetae) + desiredOmega;
%     else
%         w = k2*ye*desiredVelocity*(sin(thetae)/thetae) + k3*sin(thetae) + desiredOmega;
%     end
    
    if w > maximumOmega
        w = maximumOmega;
    elseif w < -maximumOmega
        w = -maximumOmega;
    else
        % do nothing
    end
    
%     if abs(v) < numsingTolerance
%         i = i + 5;
%     else
%         % do nothing
%     end
    
% Global Frame Error
    if i == 1 || i == 2%length(xRef)
        xE = xRef(i,:) - xAct;
    else
        xE = xRef(i-2,:) - xAct;
    end
%     yE = yRef(i,:) - yAct;
%     thetaE = thetaRef(i,:) - thetaAct;
end
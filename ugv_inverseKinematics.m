function [steeringAngle, wheelVelocity] = ugv_inverseKinematics(velocity,omega,Lf,Lr,W)
% UGV Inverse Kinematics

% Calculate front and rear steering angle
%     if abs(velocity) > 1e-12
       frontSteerAngle = atan(omega*(Lf+Lr)/sqrt(velocity^2 + 0^2));
%     else
%        frontSteerAngle = 0;
%     end

    if frontSteerAngle > deg2rad(60)
        frontSteerAngle = deg2rad(60);
    elseif frontSteerAngle < deg2rad(-60)
        frontSteerAngle = deg2rad(-60);
    else
       % do nothing
    end
    rearSteerAngle = -frontSteerAngle;
    
% Instantaneous Center of Rotation (ICR)
    R = (Lf + Lr)/(2*tan(frontSteerAngle));
    Ri = R - W/2;
    Ro = R + W/2;
% Individual wheel steering angle
    delta1 = atan((Lf+Lr)/(2*(Ri))); % front wheel 1
    delta2 = atan((Lf+Lr)/(2*(Ro))); % front wheel 2
    delta3 = -delta2; % front wheel 3
    delta4 = -delta1; % front wheel 4
    
% Calculate front and rear angular     
%     frontwheelOmega = velocity/(wheelradius*cos(frontSteerAngle));
%     rearwheelOmega = velocity/wheelradius;
    if R == inf
        velocity1 = velocity/cos(delta1);
        velocity2 = velocity/cos(delta2);
%         velocity3 = velocity/cos(delta3);
%         velocity4 = velocity/cos(delta4);
    else
        velocity1 = velocity*Ri/(R*cos(delta1));
        velocity2 = velocity*Ro/(R*cos(delta2));
%         velocity3 = velocity*Ro/(R*cos(delta3));
%         velocity4 = velocity*Ri/(R*cos(delta4));
    end
    
%     if velocity1 > 0.5
%         velocity1 = 0.5;
%     else
%     end
%     if velocity2 > 0.5
%         velocity2 = 0.5;
%     else
%     end
    velocity3 = velocity2;
    velocity4 = velocity1;
% Output
    steeringAngle = [delta1; delta2; delta3; delta4];
    wheelVelocity = [velocity1; velocity2; velocity3; velocity4];
end
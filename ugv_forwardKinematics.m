function [velocity,omega] = ugv_forwardKinematics(steeringAngle,wheelVelocity,Lf,Lr,W)
% UGV Forward Kinematics

    vel1 = wheelVelocity(1);
    vel2 = wheelVelocity(2);
    vel3 = wheelVelocity(3);
    vel4 = wheelVelocity(4);
    
    delta1 = steeringAngle(1);
    delta2 = steeringAngle(2);
    delta3 = steeringAngle(3);
    delta4 = steeringAngle(4);
    
    velx = (1/4)*(vel1*cos(delta1) + vel2*cos(delta2) +...
            vel3*cos(delta3) + vel4*cos(delta4));
    vely = (1/4)*(vel1*sin(delta1) + vel2*sin(delta2) +...
            vel3*sin(delta3) + vel4*sin(delta4));
    omega = (vel1*((Lf*sin(delta1) - (W/2)*cos(delta1))/(4*Lf^2 + 4*(W/2)^2)) +...
             vel2*((Lf*sin(delta2) + (W/2)*cos(delta2))/(4*Lf^2 + 4*(-W/2)^2)) +...
             vel3*((-Lr*sin(delta3) + (W/2)*cos(delta3))/(4*(-Lr)^2 + 4*(-W/2)^2)) +...
             vel4*((-Lr*sin(delta4) - (W/2)*cos(delta4))/(4*(-Lr)^2 + 4*(W/2)^2)));
    
%     omega = (1/4)*(vel1*sin(delta1) + vel2*sin(delta2) - vel3*sin(delta3) - vel4*sin(delta4))/...
%             (Lf + Lr);
    if vely < 1
        velocity = velx;
    else
        velocity = velx;
    end
end
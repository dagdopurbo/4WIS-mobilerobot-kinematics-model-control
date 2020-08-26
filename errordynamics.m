function xdot = errordynamics(t,x,desiredVelocity,desiredOmega,controllerGain)
    k1 = controllerGain(1);
    k2 = controllerGain(2);
    k3 = controllerGain(3);
%% First Model
    xdot(1) = -k1*x(1) + desiredOmega*x(2) + k3*x(2)*sin(x(3)) + (1/k2)*desiredVelocity*x(2)^2;
    xdot(2) = -(1/k2)*x(1)*x(2)*desiredVelocity - k3*x(1)*sin(x(3)) - desiredOmega*x(1) + desiredVelocity*sin(x(3));
    xdot(3) = -(1/k2)*desiredVelocity*x(2) - k3*sin(x(3));
%% Second Model
%     if x(3) == 0
%         lamda = 1;
%     else
%         lamda = sin(x(3))/x(3);
%     end
%     xdot(1) = -k1*x(1) + desiredOmega*x(2) + k3*x(2)*sin(x(3)) + k2*desiredVelocity*(lamda)*x(2)^2;
%     xdot(2) = -k2*desiredVelocity*(lamda)*x(2)*x(1) - desiredOmega*x(1) - k3*x(1)*sin(x(3)) + desiredVelocity*sin(x(3));
%     xdot(3) = -k2*desiredVelocity*(lamda)*x(2) - k3*sin(x(3));
    xdot = [xdot(1);xdot(2);xdot(3)];
end
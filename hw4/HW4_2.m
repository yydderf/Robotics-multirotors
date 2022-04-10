data = readtable("HW4-2.xls");
data = data{:, :};
time = 1:length(data);


quaternion = zeros(length(data), 4);
quaternion(1,:) = [1 0 0 0];
beta = 0.033;

for t = 1:length(data)-1
    q = quaternion(t,:);
    acc = data(t,:) / norm(data(t,:));

    F = [2*(q(2)*q(4) - q(1)*q(3)) - acc(1)
         2*(q(1)*q(2) + q(3)*q(4)) - acc(2)
         2*(0.5 - q(2)^2 - q(3)^2) - acc(3)];
    J = [-2*q(3),	 2*q(4),    -2*q(1),	2*q(2)
         2*q(2),     2*q(1),     2*q(4),	2*q(3)
         0,         -4*q(2),    -4*q(3),	0    ];
    step = (J'*F);
    step = step / norm(step);

    %qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';
    q = q - beta * step';
    quaternion(t+1,:) = q / norm(q); % normalise quaternion
end

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', "accelerometer");
hold on;
plot(time, data(:,1), 'r');
plot(time, data(:,2), 'g');
plot(time, data(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('MadgwickAHRS Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

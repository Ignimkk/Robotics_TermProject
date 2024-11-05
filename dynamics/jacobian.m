robot = robot_parameters();

disp('Robot Body Names:');
disp(robot.BodyNames);

endEffectorName = robot.BodyNames{end}; 

% 목표 각도 설정
targetJointAngles = [pi/6; pi/4; pi/3; pi/6];  % 임의의 각도 설정

% Jacobian 계산
jacobianMatrix = geometricJacobian(robot, targetJointAngles, endEffectorName);

% Jacobian 행렬 출력
disp('Jacobian at Target Position:');
disp(jacobianMatrix);

figure;

subplot(1, 2, 1);
imagesc(abs(jacobianMatrix)); 
colorbar;
title('Jacobian Matrix Visualization');
xlabel('Joint Variables');
ylabel('End-Effector Velocity Components');

subplot(1, 2, 2);
quiver3(0, 0, 0, jacobianMatrix(1,1), jacobianMatrix(2,1), jacobianMatrix(3,1), 'r');
hold on;
quiver3(0, 0, 0, jacobianMatrix(1,2), jacobianMatrix(2,2), jacobianMatrix(3,2), 'g');
quiver3(0, 0, 0, jacobianMatrix(1,3), jacobianMatrix(2,3), jacobianMatrix(3,3), 'b');
quiver3(0, 0, 0, jacobianMatrix(1,4), jacobianMatrix(2,4), jacobianMatrix(3,4), 'm');
title('Jacobian Velocity Vectors');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');
hold off;

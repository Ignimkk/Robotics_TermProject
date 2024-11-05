% 조인트 각도 정의
jointAngles = [pi/6; pi/6; pi/4; pi/8];

% 로봇 객체 생성 및 DH 파라미터 설정
robot = robot_parameters();
numJoints = length(jointAngles);
joint_positions = zeros(numJoints+1, 3);

T = eye(4);  
joint_positions(1, :) = T(1:3, 4)';  % 첫 위치 설정

for i = 1:numJoints
    joint_name = robot.BodyNames{i};
    T = T * getTransform(robot, jointAngles, joint_name, 'base');
    joint_positions(i+1, :) = T(1:3, 4)';  % 조인트 위치 저장
end

% 엔드 이펙터 위치 표시
endEffectorPos = T; 
joint_positions(end, :) = endEffectorPos(1:3, 4)';

figure;
plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), '-o', 'LineWidth', 2);
hold on;

for i = 1:numJoints
    theta_text = sprintf('\\theta_%d = %.2f rad', i, jointAngles(i));
    text(joint_positions(i,1), joint_positions(i,2), joint_positions(i,3), theta_text, 'Color', 'b', 'FontSize', 10, 'HorizontalAlignment', 'right');
end

end_effector_text = sprintf('End Effector Position\n(x, y, z) = (%.2f, %.2f, %.2f)', endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4));
text(endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4), end_effector_text, 'Color', 'r', 'FontSize', 10);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Arm with Joint Angles and End Effector Position');
grid on;
axis equal;
hold off;

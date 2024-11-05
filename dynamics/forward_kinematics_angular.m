% 로봇 생성
robot = robot_parameters();

% 목표 위치와 목표 자세 설정 (임의의 값 설정)
targetPos = [0.6, 0.4, 0.5];  % 목표 위치 (x, y, z)
targetOrientation = eye(3);  % 목표 자세 (3x3 단위 행렬 = 정렬된 자세)

% 역기구학을 이용하여 목표 위치에 대한 각도 추출
jointAngles = inverse_kinematics(robot, targetPos, targetOrientation);

% 각 조인트 위치 계산
numJoints = length(jointAngles);
joint_positions = zeros(numJoints+1, 3);  % 각 조인트와 엔드 이펙터 위치 저장

% 초기 위치 (베이스 위치)
T = eye(4);  % 베이스 좌표계
joint_positions(1, :) = T(1:3, 4)';  % 첫 위치 설정

% 각 조인트 위치 계산
for i = 1:numJoints
    % 현재 조인트 각도를 적용한 변환 행렬 계산
    joint_name = robot.BodyNames{i};
    T = T * getTransform(robot, jointAngles, joint_name, 'base');
    joint_positions(i+1, :) = T(1:3, 4)';  % 조인트 위치 저장
end

% 엔드 이펙터 위치 표시
endEffectorPos = T;  % 최종 변환 행렬이 엔드 이펙터 위치
joint_positions(end, :) = endEffectorPos(1:3, 4)';

% 3D 그래프 생성
figure;
plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), '-o', 'LineWidth', 2);
hold on;

% 각 조인트 위치에 theta 값을 텍스트로 표시
for i = 1:numJoints
    theta_text = sprintf('\\theta_%d = %.2f rad', i, jointAngles(i));
    text(joint_positions(i,1), joint_positions(i,2), joint_positions(i+1,3), theta_text, 'Color', 'b', 'FontSize', 10, 'HorizontalAlignment', 'right');
end

% 엔드 이펙터 위치 표시
end_effector_text = sprintf('End Effector Position\n(x, y, z) = (%.2f, %.2f, %.2f)', endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4));
text(endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4), end_effector_text, 'Color', 'r', 'FontSize', 10);

% 그래프 라벨과 설정
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Arm with Inverse Kinematics Solution');
grid on;
axis equal;
hold off;

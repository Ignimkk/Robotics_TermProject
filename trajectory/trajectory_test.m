robot = robot_parameters();

initialJointAngles = [0, 0, 0, 0]';

figure;
show(robot, initialJointAngles, 'Frames', 'on');
title('Robot Visualization in Initial Position');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
axis equal;
view(135, 30);  
hold on;

startPos = [200, 800, 700];  % 시작 위치 (m)
endPos = [600, 800, 700];    % 목표 위치 (m)
numPoints = 10;  % 경로를 세분화할 지점 수
trajectory = linspace(0, 1, numPoints);  % 0에서 1까지 비율을 나타내는 값
waypoints = (1 - trajectory') * startPos + trajectory' * endPos;  % 시작에서 목표까지 직선 경로

plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b--', 'LineWidth', 1);  % 전체 궤적 표시
currentPath = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);  % 이동 경로 초기화

for i = 1:numPoints
    targetPos = waypoints(i, :);
    targetOrientation = eye(3); 
    
    jointAngles = inverse_kinematics(robot, targetPos, targetOrientation);
    
    show(robot, jointAngles(:), 'Frames', 'on', 'PreservePlot', false);
    
    set(currentPath, 'XData', waypoints(1:i,1), 'YData', waypoints(1:i,2), 'ZData', waypoints(1:i,3));
    
    plot3(waypoints(i,1), waypoints(i,2), waypoints(i,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    
    pause(0.05); 
end
hold off;

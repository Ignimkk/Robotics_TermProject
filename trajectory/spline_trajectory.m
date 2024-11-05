robot = robot_parameters();

jointLimits = [
    -180, 180;  
     0, 90;     
     0, 90;     
     0, 90       
] * pi / 180;   

startPositions = [
    0, 800, 700

];
endPositions = [
    200, 800, 500
];
numTrajectories = size(startPositions, 1);
numPoints = 30;              

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1]; 
initialGuess = homeConfiguration(robot);  

figure;
hold on;

jointMarkers = gobjects(4, 1);  % 4개의 조인트 핸들
for j = 1:4
    jointMarkers(j) = plot3(nan, nan, nan, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
end

for t = 1:numTrajectories
    startPos = startPositions(t, :);
    endPos = endPositions(t, :);
    
    % 5차 다항식을 이용한 부드러운 궤적 생성
    time = linspace(0, 1, numPoints);
    startVel = [0, 0, 0];
    endVel = [0, 0, 0];
    startAcc = [0, 0, 0];
    endAcc = [0, 0, 0];
    
    a0 = startPos;
    a1 = startVel;
    a2 = 0.5 * startAcc;
    a3 = (20 * endPos - 20 * startPos - (8 * endVel + 12 * startVel) + (3 * startAcc - endAcc)) / 2;
    a4 = (30 * startPos - 30 * endPos + (14 * endVel + 16 * startVel) - (3 * startAcc - 2 * endAcc)) / 2;
    a5 = (12 * endPos - 12 * startPos - (6 * endVel + 6 * startVel) + (startAcc - endAcc)) / 2;

    waypoints = zeros(numPoints, 3);
    for i = 1:numPoints
        t = time(i);
        waypoints(i, :) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5;
    end

    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b--', 'LineWidth', 1);  % 전체 궤적 표시
    
    currentPath = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);

    for i = 1:numPoints
        targetPos = waypoints(i, :);
        targetOrientation = eye(3);  

        jointAngles = inverse_kinematics(robot, targetPos, targetOrientation);
        
        show(robot, jointAngles(:), 'Frames', 'on', 'PreservePlot', false);

        set(currentPath, 'XData', waypoints(1:i,1), 'YData', waypoints(1:i,2), 'ZData', waypoints(1:i,3));

        for j = 1:4
            transformMatrix = getTransform(robot, jointAngles, robot.BodyNames{j}, 'base');
            jointPosition = transformMatrix(1:3, 4)';  % 현재 관절 위치 좌표 추출
            set(jointMarkers(j), 'XData', jointPosition(1), 'YData', jointPosition(2), 'ZData', jointPosition(3));
        end

        plot3(waypoints(i,1), waypoints(i,2), waypoints(i,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

        pause(0.05);  
    end
end
hold off;

% 그래프 설정
title('곽 그리기');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
grid on;
axis equal;
view(135, 30);  % 시점 고정

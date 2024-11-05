robot = robot_parameters();

jointLimits = [
    -180, 180;   % 조인트 1: -180 ~ 180도
     0, 90;      % 조인트 2: 0 ~ 90도
     0, 90;      % 조인트 3: 0 ~ 90도
     0, 90       % 조인트 4: 0 ~ 90도
] * pi / 180;   % 각도 제한을 라디안으로 변환

startPositions = [
    0, 800, 700;
    200, 800, 700;
    100, 800, 600;
    -50, 800, 450;
    300, 800, 750;
    300, 800, 500;
    0, 800, 400;
    200, 800, 400;

];
endPositions = [
    200, 800, 700;
    200, 800, 500;
    100, 800, 450;
    250, 800, 450;
    300, 800, 300;
    400, 800, 500;
    200, 800, 400;
    200, 800, 200;

];
numTrajectories = size(startPositions, 1);
numPoints = 3;             
trajectory = linspace(0, 1, numPoints);  

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1];  
initialGuess = homeConfiguration(robot); 

figure;
hold on;

jointMarkers = gobjects(4, 1);  
for j = 1:4
    jointMarkers(j) = plot3(nan, nan, nan, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
end

for t = 1:numTrajectories
    startPos = startPositions(t, :);
    endPos = endPositions(t, :);
    waypoints = (1 - trajectory') * startPos + trajectory' * endPos; 

    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b--', 'LineWidth', 1); 
    
    currentPath = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);

    for i = 1:numPoints
        % 현재 목표 위치
        targetPos = waypoints(i, :);
        targetOrientation = eye(3); 

        jointAngles = inverse_kinematics(robot, targetPos, targetOrientation);
        
        show(robot, jointAngles(:), 'Frames', 'on', 'PreservePlot', false);

        set(currentPath, 'XData', waypoints(1:i,1), 'YData', waypoints(1:i,2), 'ZData', waypoints(1:i,3));

        for j = 1:4
            transformMatrix = getTransform(robot, jointAngles, robot.BodyNames{j}, 'base');
            jointPosition = transformMatrix(1:3, 4)';  
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

% 각 축의 제한 설정
xlim([-500, 1500]);
ylim([-500, 1500]);
zlim([-500, 1500]);

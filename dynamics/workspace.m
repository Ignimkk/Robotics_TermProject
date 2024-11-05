robot = robot_parameters();

joint1Range = linspace(-90, 90, 10) * pi / 180;  % -180°에서 180°까지 (라디안으로 변환)
joint2Range = linspace(0, 90, 5) * pi / 180;       % 0°에서 90°
joint3Range = linspace(0, 90, 5) * pi / 180;       % 0°에서 90°
joint4Range = linspace(0, 90, 5) * pi / 180;       % 0°에서 90°

workspacePoints = [];

for theta1 = joint1Range
    for theta2 = joint2Range
        for theta3 = joint3Range
            for theta4 = joint4Range
                jointAngles = [theta1, theta2, theta3, theta4];
                
                endEffectorPos = getTransform(robot, jointAngles', robot.BodyNames{end}, 'base');
                
                workspacePoints = [workspacePoints; endEffectorPos(1:3, 4)'];
            end
        end
    end
end

figure;
scatter3(workspacePoints(:,1), workspacePoints(:,2), workspacePoints(:,3), 'filled');
title('Robot Arm Workspace');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
grid on;
axis equal;
view(135, 30);  % 시점 설정

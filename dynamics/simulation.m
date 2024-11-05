clc;
clear;
clearvars;
clf;

robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 5);

L1 = 270;  % Link 1 고정
L2 = 236;
L3 = 500;
L4 = 460;
L5 = 195;

% Link 1 (Base, Fixed)
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1, trvec2tform([0, 0, L1]));
body1.Joint = joint1;
addBody(robot, body1, 'base');

% Link 2
body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2, trvec2tform([0, 0, L2]) * rotm2tform(rotx(pi/2)));
body2.Joint = joint2;
addBody(robot, body2, 'link1');

% Link 3
body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint3, trvec2tform([L3, 0, 0]) * rotm2tform(rotz(pi/2)));
body3.Joint = joint3;
addBody(robot, body3, 'link2');

% Link 4
body4 = rigidBody('link4');
joint4 = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(joint4, trvec2tform([L4, 0, 0]));
body4.Joint = joint4;
addBody(robot, body4, 'link3');

% Link 5 (End Effector)
body5 = rigidBody('endeffector');
joint5 = rigidBodyJoint('joint5', 'fixed');
setFixedTransform(joint5, trvec2tform([L5, 0, 0]));
body5.Joint = joint5;
addBody(robot, body5, 'link4');

figure;
hold on;

u_vals = linspace(0, 10, 20);

% 궤적 계산
trajectory = zeros(length(u_vals), 3); % X, Y, Z 좌표 저장
for i = 1:length(u_vals)
    u = u_vals(i);
    [xd, yd] = Desired_Trajectory(u);
    trajectory(i, :) = [xd * 500, yd * 500, 700]; % 스케일 조정
end

plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b--', 'LineWidth', 1);

for i = 1:size(trajectory, 1)
    targetPos = trajectory(i, :);
    tform = trvec2tform(targetPos);  
    config = ikSolver(robot, tform); 

    show(robot, config, 'Frames', 'on', 'PreservePlot', false);
    plot3(targetPos(1), targetPos(2), targetPos(3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    
    pause(0.1);
end

hold off;

title('Desired Trajectory 궤적 추적 및 RigidBodyTree 기반 로봇 이동');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
axis equal;
view(135, 30);

function [xd, yd] = Desired_Trajectory(u)
    xd = 1 + 0.5 * sin((2 * pi / 5) * u + pi / 2);
    yd = 1 + 0.5 * cos((2 * pi / 5) * u + pi / 2);
end

% 역기구학 솔버 함수 정의
function config = ikSolver(robot, tform)
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25, 0.25, 0.25, 1, 1, 1]; % 위치 및 방향 가중치 설정
    initialGuess = robot.homeConfiguration;
    [config, ~] = ik('endeffector', tform, weights, initialGuess);
end

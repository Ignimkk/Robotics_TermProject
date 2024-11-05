robot = robot_parameters();

initial_joint_angles = [0; pi/4; pi/4; pi/4];  % 시작 위치
target_joint_angles = [pi/6; pi/4; pi/3; pi/6];  % 목표 위치

num_steps = 50;
angles_trajectory = zeros(4, num_steps);
for j = 1:4
    angles_trajectory(j, :) = linspace(initial_joint_angles(j), target_joint_angles(j), num_steps);
end

figure;
hold on;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('End-Effector Velocity Directions and Singularity Analysis');
axis equal;

scale = 50;

for i = 1:num_steps
    joint_angles = angles_trajectory(:, i);
    
    % Jacobian 행렬 계산
    J = geometricJacobian(robot, joint_angles, robot.BodyNames{end});
    
    end_effector_pos = getTransform(robot, joint_angles, robot.BodyNames{end}, 'base');
    end_effector_pos = end_effector_pos(1:3, 4);  % 위치 벡터
    
    for j = 1:4
        joint_velocity = zeros(4, 1);
        joint_velocity(j) = 1;  % 특정 관절만 움직이는 속도 벡터
        
        end_effector_velocity = J * joint_velocity;
        
        quiver3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), ...
                end_effector_velocity(1)*scale, end_effector_velocity(2)*scale, end_effector_velocity(3)*scale, ...
                'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'Color', 'b');
    end
    
    if rank(J) < min(size(J))  % 특이점일 경우
        plot3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    else
        plot3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'go', 'MarkerSize', 8);
    end
    
    pause(0.1); 
end

legend({'Velocity Vectors', 'Non-Singularity Position', 'Singularity Position'}, 'Location', 'Best');
hold off;

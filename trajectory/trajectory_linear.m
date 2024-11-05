robot = robot_parameters();  

initial_joint_angles = [0; 0; 0; 0];  % 시작 위치
target_joint_angles = [pi/6; pi/4; pi/3; pi/6];  % 목표 위치

total_time = 5; 
num_samples = 100;  
time_vector = linspace(0, total_time, num_samples);

q_traj = zeros(4, num_samples);
for j = 1:4
    q_traj(j, :) = linspace(initial_joint_angles(j), target_joint_angles(j), num_samples);
end

% 그래프에서 각 궤적 확인
figure;
subplot(3,1,1);
plot(time_vector, q_traj');
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');

% 로봇의 동작 시뮬레이션
figure;
for i = 1:num_samples
    joint_angles = q_traj(:, i);

    show(robot, joint_angles, 'PreservePlot', false);
    title(['Robot Motion - Time: ' num2str(time_vector(i), '%.2f') ' s']);
    hold on;
    
    for j = 1:4
        linkTransform = getTransform(robot, joint_angles, robot.BodyNames{j}, 'base');
        position = linkTransform(1:3, 4); 
        
        text(position(1), position(2), position(3), ...
             ['\theta_' num2str(j) ' = ' num2str(rad2deg(joint_angles(j)), '%.1f') '°'], ...
             'FontSize', 10, 'Color', 'b', 'HorizontalAlignment', 'center');
    end
    hold off;
    
    drawnow;
    pause(0.05);
end

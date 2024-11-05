robot = robot_parameters(); 

initial_joint_angles = [0; 0; 0; 0];  % 시작 위치
target_joint_angles = [pi/6; pi/4; pi/3; pi/6];  % 목표 위치

total_time = 5;  
num_samples = 100; 
time_vector = linspace(0, total_time, num_samples);

[q_traj, qd_traj, qdd_traj] = trapveltraj([initial_joint_angles, target_joint_angles], num_samples, 'EndTime', total_time);

figure;
subplot(3,1,1);
plot(time_vector, q_traj');
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');

subplot(3,1,2);
plot(time_vector, qd_traj');
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');

subplot(3,1,3);
plot(time_vector, qdd_traj');
title('Joint Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');

% 로봇의 동작 시뮬레이션
for i = 1:num_samples
    joint_angles = q_traj(:, i);

    show(robot, joint_angles, 'PreservePlot', false);
    title(['Robot Motion - Time: ' num2str(time_vector(i), '%.2f') ' s']);
    drawnow;
    pause(0.05); 
end

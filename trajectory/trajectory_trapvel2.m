robot = robot_parameters(); 

% 시작 및 목표 위치 정의 (각 관절의 초기 각도와 목표 각도)
initial_joint_angles = [0; 0; 0; 0];  % 시작 위치
target_joint_angles = [pi/6; pi/4; pi/3; pi/6];  % 목표 위치

% 궤적 시간 설정
total_time = 5;  % 전체 시간 (초)
num_samples = 100;  % 샘플 수
time_vector = linspace(0, total_time, num_samples);

% 트래페 프로파일 생성
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
             'FontSize', 10, 'Color', 'r', 'HorizontalAlignment', 'center');
    end
    hold off;
    
    drawnow;
    pause(0.05); 
end

modelName = 'RobotTrajectoryTracking';
open_system(new_system(modelName));

set_param(modelName, 'Solver', 'ode45', 'StopTime', '10');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Desired_Trajectory']);
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Inverse_Kinematicss']);
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Forward_Kinematic']);

% MATLAB Function 블록에 수동으로 입력할 코드
% 1. Desired Trajectory
% function [xd, yd] = Desired_Trajectory(u)
% xd = 1 + 0.5 * sin((2 * pi / 5) * u + pi / 2);
% yd = 1 + 0.5 * cos((2 * pi / 5) * u + pi / 2);
% end

% 2. Inverse Kinematics
% function [theta1d, theta2d, theta3d, theta4d] = Inverse_Kinematicss(xd, yd)
% l1 = 1; l2 = 2; l3 = 1.5; l4 = 1;
% theta2d = acos((xd^2 + yd^2 - l1^2 - l2^2) / (2 * l1 * l2));
% temp = (l1 + l2 * cos(theta2d));
% theta1d = atan2(yd, xd) - atan2((l2 * sin(theta2d)), temp);
% x = temp * cos(theta1d); y = temp * sin(theta1d); z = yd - l2 * sin(theta2d);
% theta4d = acos((x^2 + y^2 + z^2 - l3^2 - l4^2) / (2 * l3 * l4));
% num = l3^2 + l4^2 - (x^2 + y^2 + z^2); denom = 2 * l3 * l4;
% a = sqrt(x^2 + y^2); p = (num / denom)^2; b = sqrt(p - 1);
% theta3d = atan2(z, a) - atan2(b, num / denom);
% end

% 3. Forward Kinematics
% function [xa, ya] = Forward_Kinematic(theta1_a, theta2_a, theta3_a, theta4_a)
% l1 = 1; l2 = 1; l3 = 1.5; l4 = 1;
% xa = l1 * cos(theta1_a) + l2 * cos(theta1_a + theta2_a) + l3 * cos(theta1_a + theta2_a + theta3_a) + l4 * cos(theta1_a + theta2_a + theta3_a + theta4_a);
% ya = l1 * sin(theta1_a) + l2 * sin(theta1_a + theta2_a) + l3 * sin(theta1_a + theta2_a + theta3_a) + l4 * sin(theta1_a + theta2_a + theta3_a + theta4_a);
% end

% PID Controller 블록 생성
for i = 1:4
    pidName = ['PID' num2str(i)];
    add_block('simulink/Continuous/PID Controller', [modelName '/' pidName]);
    set_param([modelName '/' pidName], 'P', '10', 'I', '1', 'D', '0.1');
end

add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Robot']);
for i = 1:4
    add_block('simulink/Commonly Used Blocks/In1', [modelName '/Robot/JointInput' num2str(i)]);
    add_block('simulink/Commonly Used Blocks/Out1', [modelName '/Robot/JointOutput' num2str(i)]);
end

add_block('simulink/Commonly Used Blocks/Scope', [modelName '/O_P_Graph']);

add_line(modelName, 'Desired_Trajectory/1', 'Inverse_Kinematicss/1');
add_line(modelName, 'Inverse_Kinematicss/1', 'PID1/1');
add_line(modelName, 'Inverse_Kinematicss/2', 'PID2/1');
add_line(modelName, 'Inverse_Kinematicss/3', 'PID3/1');
add_line(modelName, 'Inverse_Kinematicss/4', 'PID4/1');

for i = 1:4
    add_line(modelName, ['PID' num2str(i) '/1'], ['Robot/JointInput' num2str(i)]);
end

for i = 1:4
    add_line(modelName, ['Robot/JointOutput' num2str(i)], ['Forward_Kinematic/' num2str(i)]);
end

add_line(modelName, 'Forward_Kinematic/1', 'O_P_Graph/1');
add_line(modelName, 'Forward_Kinematic/2', 'O_P_Graph/1');

save_system(modelName);

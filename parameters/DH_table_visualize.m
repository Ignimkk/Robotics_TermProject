clc;
clear all;
clearvars;
clf;
syms theta1 theta2 d3 theta4;

% Define link lengths
L1 = 300;  % Link 1
L2 = 300;  % Link 2
d3_val = 100; % Vertical linear movement for prismatic joint (along Z-axis)
L4 = 100;  % End-effector link length

% Initial joint angles
theta1_val = 0;  % Joint 1 (rotation around Z-axis)
theta2_val = 0;  % Joint 2 (rotation around Z-axis)
theta4_val = 0;  % Joint 4 (rotation around Z-axis)

% DH Parameters for SCARA robot
a1 = L1; alpha1 = 0; d1 = 0; % Joint 1 (rotational in XY plane)
a2 = L2; alpha2 = 0; d2 = 0; % Joint 2 (rotational in XY plane)
a3 = 0; alpha3 = 0; d3 = d3_val; % Joint 3 (prismatic joint in Z-axis)
a4 = L4; alpha4 = 0; d4 = 0; % Joint 4 (rotational in XY plane)

% Transformation matrices based on DH parameters
H0_1 = DH(a1, alpha1, 0, theta1_val); % Rotation at Joint 1
H1_2 = DH(a2, alpha2, 0, theta2_val); % Rotation at Joint 2
H2_3 = DH(0, 0, d3, 0);               % Prismatic joint for vertical movement
H3_4 = DH(a4, alpha4, 0, theta4_val); % End-effector rotation

% Cumulative transformation matrices
H0_2 = H0_1 * H1_2;
H0_3 = H0_2 * H2_3;
H0_4 = H0_3 * H3_4; % Full transformation to end-effector

% Extract link positions
O0 = [0, 0, 0];
O1 = H0_1(1:3, 4)'; % After Joint 1
O2 = H0_2(1:3, 4)'; % After Joint 2
O3 = H0_3(1:3, 4)'; % Prismatic joint movement
O4 = H0_4(1:3, 4)'; % End-effector

% Plot the SCARA robot in static pose
figure;
hold on;

% Define joint size
joint_radius = 30; % Adjust as needed for visualization
joint_height = 50; % Height of the joint cylinders
[zc, xc, yc] = cylinder(joint_radius);

% Plot Links as Lines
plot3([O0(1), O1(1)], [O0(2), O1(2)], [O0(3), O1(3)], 'r', 'LineWidth', 3); % Link 1
plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'g', 'LineWidth', 3); % Link 2
plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'b', 'LineWidth', 3); % Prismatic link (Z-axis movement)
plot3([O3(1), O4(1)], [O3(2), O4(2)], [O3(3), O4(3)], 'm', 'LineWidth', 3); % End-effector link

% Add Cylindrical Joints at Each Joint Position
surf(xc + O1(1), yc + O1(2), zc * joint_height + O1(3), 'FaceColor', 'r', 'EdgeColor', 'none');
surf(xc + O2(1), yc + O2(2), zc * joint_height + O2(3), 'FaceColor', 'g', 'EdgeColor', 'none');
surf(xc + O3(1), yc + O3(2), zc * joint_height + O3(3), 'FaceColor', 'b', 'EdgeColor', 'none');
surf(xc + O4(1), yc + O4(2), zc * joint_height + O4(3), 'FaceColor', 'm', 'EdgeColor', 'none');

% Display end-effector position
txtend = ['x = ', num2str(O4(1)), ' , y = ', num2str(O4(2)), ' , z = ', num2str(O4(3))];
text(O4(1), O4(2), O4(3), txtend, 'FontSize', 8, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% Set plot attributes
title('SCARA Robot Visualization');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
xlim([-1000, 1000]);
ylim([-1000, 1000]);
zlim([0, 1000]);
grid on;
axis equal;

% DH transformation matrix function
function [A] = DH(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

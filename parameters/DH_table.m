clc;
clear all;
clearvars;
clf;
syms theta1 theta2 theta3 theta4;

L1 = 270;  % Link 1 is fixed
L2 = 236;
L3 = 500;
L4 = 460;
L5 = 195; % End-effector

theta1_val = 0;  
theta2_val = 0;
theta3_val = 0;
theta4_val = 0;

a1 = 0; alpha1 = 0; d1 = L1; theta1_offset = 0;
a2 = 0; alpha2 = pi/2; d2 = L2; theta2_offset = 0;
a3 = L3; alpha3 = 0; d3 = 0; theta3_offset = pi/2;
a4 = L4; alpha4 = 0; d4 = 0; theta4_offset = 0;
a5 = L5; alpha5 = 0; d5 = 0; theta5_offset = 0;

H0_1 = DH(a1, alpha1, d1, theta1_offset);  % Link 1 is fixed
H1_2 = DH(a2, alpha2, d2, theta1_val + theta2_offset); % Joint 1 rotates around Z-axis between link-1 and link-2
H2_3 = DH(a3, alpha3, d3, theta2_val + theta3_offset);
H3_4 = DH(a4, alpha4, d4, theta3_val + theta4_offset);
H4_5 = DH(a5, alpha5, d5, theta4_val); % End Effector has no rotation

H0_2 = H0_1 * H1_2;
H0_3 = H0_2 * H2_3;
H0_4 = H0_3 * H3_4;
H0_5 = H0_4 * H4_5; 

O0 = [0, 0, 0];
O1 = H0_1(1:3, 4)'; % Link 1 (fixed base)
O2 = H0_2(1:3, 4)'; % After Joint 1 (between link 1 and 2)
O3 = H0_3(1:3, 4)';
O4 = H0_4(1:3, 4)';
O5 = H0_5(1:3, 4)';

figure;
plot3([O0(1), O1(1)], [O0(2), O1(2)], [O0(3), O1(3)], 'k', 'LineWidth', 3); % Fixed link 1
hold on;
plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'r', 'LineWidth', 3); % Link 2
plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'b', 'LineWidth', 3); % Link 3
plot3([O3(1), O4(1)], [O3(2), O4(2)], [O3(3), O4(3)], 'g', 'LineWidth', 3); % Link 4
plot3([O4(1), O5(1)], [O4(2), O5(2)], [O4(3), O5(3)], 'm', 'LineWidth', 3); % End Effector

txtend = ['x = ', num2str(O5(1)), ' , y = ', num2str(O5(2)), ' , z = ', num2str(O5(3))];
text(O5(1), O5(2), O5(3), txtend, 'FontSize', 8, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

title('5 DOF Robotic Arm Visualization (Link 1 Fixed)');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
xlim([-3000, 3000]);
ylim([-3000, 3000]);
zlim([0, 5000]);
grid on;
axis equal;

function [A] = DH(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

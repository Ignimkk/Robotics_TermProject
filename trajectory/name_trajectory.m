imageWidth = 500;
imageHeight = 500;
textImage = insertText(zeros(imageHeight, imageWidth), [400, 500], 'R', 'FontSize', 50, 'BoxOpacity', 0);
grayImage = rgb2gray(textImage);  % 회색조 변환

bwImage = imbinarize(grayImage);
edgeImage = edge(bwImage, 'Canny');

[yPoints, xPoints] = find(edgeImage);  
if isempty(xPoints) || isempty(yPoints)
    error('윤곽선 좌표가 생성되지 않았습니다. 글자 이미지 또는 윤곽선 검출을 확인하세요.');
end

scaleFactor = 2;  
zPos = 300;  % 고정된 높이 (Z 좌표)
letter_points = [xPoints * scaleFactor, yPoints * scaleFactor, zPos * ones(size(xPoints))];

disp('추출된 좌표:');
disp(letter_points);

robot = robot_parameters();

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1];
initialGuess = homeConfiguration(robot);

figure;
hold on;
currentPath = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);

for i = 1:size(letter_points, 1)
    targetPos = letter_points(i, :);
    targetOrientation = eye(3);

    jointAngles = inverse_kinematics(robot, targetPos, targetOrientation);

    show(robot, jointAngles(:), 'Frames', 'on', 'PreservePlot', false);

    set(currentPath, 'XData', letter_points(1:i,1), 'YData', letter_points(1:i,2), 'ZData', letter_points(1:i,3));
    plot3(letter_points(i,1), letter_points(i,2), letter_points(i,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

    pause(0.01); 
end
hold off;

title('End-Effector Path Drawing Letters');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
axis equal;
view(135, 30);

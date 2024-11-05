function [theta1d, theta2d, theta3d, theta4d] = Inverse_Kinematicss(xd, yd)
    l1 = 1;
    l2 = 2;
    l3 = 1.5;
    l4 = 1;

    % theta2d 계산
    theta2d = acos((xd^2 + yd^2 - l1^2 - l2^2) / (2 * l1 * l2));

    % theta1d 계산
    temp = (l1 + l2 * cos(theta2d));
    theta1d = atan2(yd, xd) - atan2((l2 * sin(theta2d)), temp);

    % 중간 계산값
    x = temp * cos(theta1d);
    y = temp * sin(theta1d);
    z = yd - l2 * sin(theta2d);

    % theta4d 계산
    theta4d = acos((x^2 + y^2 + z^2 - l3^2 - l4^2) / (2 * l3 * l4));

    % theta3d 계산
    num = l3^2 + l4^2 - (x^2 + y^2 + z^2);
    denom = 2 * l3 * l4;
    a = sqrt(x^2 + y^2);
    p = (num / denom)^2;
    b = sqrt(p - 1);
    theta3d = atan2(z, a) - atan2(b, num / denom);
end

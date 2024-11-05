syms theta1 theta2 theta3 theta4 d1 d2 d3 d4 a1 a2 a3 a4 alpha1 alpha2 alpha3 alpha4 real

d1 = 270; a2 = 500; a3 = 460; a4 = 195;  % 사용자 로봇 팔의 링크 길이
alpha1 = 0; alpha2 = pi/2; alpha3 = 0; alpha4 = 0;  % 각 링크의 트위스트 각도

DH_table = [0,      alpha1, d1, theta1;
            a2,     alpha2, 0, theta2;
            a3,     alpha3, 0, theta3;
            a4,     alpha4, 0, theta4];

T = eye(4);
positions = sym(zeros(3, 4));  
for i = 1:4
    a = DH_table(i, 1);
    alpha = DH_table(i, 2);
    d = DH_table(i, 3);
    theta = DH_table(i, 4);
    
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
     
    T = T * A;
    
    positions(:, i) = T(1:3, 4);
end

% 엔드 이펙터 위치 벡터
end_effector_pos = positions(:, 4); 

% 자코비안 행렬 계산
J = jacobian(end_effector_pos, [theta1, theta2, theta3, theta4]);

% 자코비안 출력
disp('Symbolic Jacobian Matrix for 4-DOF Robot Arm:');
pretty(J)  % 자코비안을 출력

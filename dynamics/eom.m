function DH_table_visualize_with_physical_properties_and_ik
    syms q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 real
    syms m1 m2 m3 m4 m5 g real  
    syms I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz real
    syms c1x c1y c1z c2x c2y c2z c3x c3y c3z c4x c4y c4z c5x c5y c5z real  % 무게 중심

    % 링크1: 질량, 관성, 무게 중심
    m1 = 53.341;
    I1 = diag([0.864, 1.092, 1.189]);
    c1 = [31.881, 891.094, 398.289]*1e-3;

    % 링크2
    m2 = 39.345;
    I2 = diag([0.763, 0.835, 0.297]);
    c2 = [-1.243, 891.094, 680.381]*1e-3;

    % 링크3
    m3 = 41.857;
    I3 = diag([2.001, 1.753, 0.43]);
    c3 = [2.042, 891.094, 1002.596]*1e-3;

    % 링크4
    m4 = 24.017;
    I4 = diag([0.617, 0.62, 0.085]);
    c4 = [8.833, 911.094, 1525.873]*1e-3;

    % 링크5
    m5 = 3.195;
    I5 = diag([0.017, 0.018, 0.006]);
    c5 = [11.461, 911.094, 1787.483]*1e-3;

    q = [q1; q2; q3; q4; q5];
    dq = [dq1; dq2; dq3; dq4; dq5];
    ddq = [ddq1; ddq2; ddq3; ddq4; ddq5];

    T = 0; 
    V = 0;  

    g_vector = [0; 0; -g];

    for i = 1:5
        mass = eval(['m', num2str(i)]);
        inertia = eval(['I', num2str(i)]);
        com = eval(['c', num2str(i)]);
        
       
        r_com = com;  
        v_com = jacobian(r_com, q) * dq;  

        T = T + 0.5 * mass * (v_com.' * v_com) + 0.5 * dq(i)^2 * inertia(3,3); % 수정된 부분: 각속도에 대해 각 관성 적용
        
        V = V + mass * g_vector.' * r_com(:);
    end

    L = T - V;

    tau = sym('tau', [5, 1]);  % 토크 벡터

    for i = 1:5
        dLdqi = diff(L, dq(i));
        ddt_dLdqi = diff(dLdqi, q(i)) * dq(i) + diff(dLdqi, dq(i)) * ddq(i);  % 시간에 따른 미분
        dL_dqi = diff(L, q(i));
        
        tau(i) = ddt_dLdqi - dL_dqi;  % 오일러-라그랑주 방정식
    end

    % 결과 출력
    tau = simplify(tau);
    disp('Torque required at each joint:');
    disp(tau);
end

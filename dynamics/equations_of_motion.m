function tau = equations_of_motion(robot, jointAngles, jointVelocities, jointAccelerations)
    % 중력 가속도
    g = 9.81;  
    gravity_vector = [0; 0; -g];  

    % 초기화
    num_joints = numel(jointAngles);
    z0 = [0; 0; 1];  % 기준 z축
    tau = zeros(num_joints, 1); 

    w = cell(num_joints+1, 1);  
    wd = cell(num_joints+1, 1); 
    vd = cell(num_joints+1, 1);
    w{1} = [0; 0; 0];
    wd{1} = [0; 0; 0];
    vd{1} = gravity_vector;
    
    for i = 1:num_joints
        body = robot.Bodies{i};
        T = getTransform(robot, jointAngles, body.Name, 'base');
        R = T(1:3, 1:3); 
        
        w{i+1} = R.' * (w{i} + jointVelocities(i) * z0);
        wd{i+1} = R.' * (wd{i} + jointAccelerations(i) * z0 + cross(w{i}, jointVelocities(i) * z0));
        
        com = body.CenterOfMass(:);  
        vd{i+1} = cross(wd{i+1}, com) + cross(w{i+1}, cross(w{i+1}, com)) + vd{i};
    end

    f = cell(num_joints+1, 1);  % 힘
    n = cell(num_joints+1, 1);  % 모멘트
    f{num_joints+1} = [0; 0; 0];
    n{num_joints+1} = [0; 0; 0];
    
    for i = num_joints:-1:1
        body = robot.Bodies{i};
        mass = body.Mass;
        inertia = diag(body.Inertia(1:3));  
        com = body.CenterOfMass(:);  

        % 힘과 모멘트 계산
        f{i} = R * f{i+1} + mass * vd{i+1};
        n{i} = R * n{i+1} + cross(com, mass * vd{i+1}) + inertia * wd{i+1};
        
        % 토크 계산
        tau(i) = n{i}' * z0;
    end
end

function DH_table_visualize_with_physical_properties_and_ik
    a = [0, 0, 500, 460, 195];
    alpha = [0, pi/2, 0, 0, 0];
    d = [270, 236, 0, 0, 0];
    theta_offset = [0, 0, pi/2, 0, 0];  % 각 링크의 초기 theta 오프셋 값

    link_properties = {
        struct('mass', 53.341, 'com', [31.881, 891.094, 398.289]*1e-3, ...
               'inertia', [0.864, 1.092, 1.189, -2.22e-16, -0.115, -7.105e-15]), % 링크 1
        struct('mass', 39.345, 'com', [-1.243, 891.094, 680.381]*1e-3, ...
               'inertia', [0.763, 0.835, 0.297, -1.301e-12, -5.751e-4, -1.066e-14]), % 링크 2
        struct('mass', 41.857, 'com', [2.042, 891.094, 1002.596]*1e-3, ...
               'inertia', [2.001, 1.753, 0.43, 4.44e-12, -0.016, -2.913e-13]), % 링크 3
        struct('mass', 24.017, 'com', [8.833, 911.094, 1525.873]*1e-3, ...
               'inertia', [0.617, 0.62, 0.085, -4.266e-14, -0.009, -1.563e-13]), % 링크 4
        struct('mass', 3.195, 'com', [11.461, 911.094, 1787.483]*1e-3, ...
               'inertia', [0.017, 0.018, 0.006, -2.474e-14, -1.314e-4, -1.776e-15]) % 링크 5
    };

    % 로봇 모델 생성
    robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 5);

    % 링크와 조인트 추가
    for i = 1:5
        body = rigidBody(['body' num2str(i)]);
        jnt = rigidBodyJoint(['joint' num2str(i)], 'revolute');

        body.Mass = link_properties{i}.mass;
        body.CenterOfMass = link_properties{i}.com;
        body.Inertia = link_properties{i}.inertia;

        transform = [cos(theta_offset(i)), -sin(theta_offset(i))*cos(alpha(i)), sin(theta_offset(i))*sin(alpha(i)), a(i)*cos(theta_offset(i));
                     sin(theta_offset(i)), cos(theta_offset(i))*cos(alpha(i)), -cos(theta_offset(i))*sin(alpha(i)), a(i)*sin(theta_offset(i));
                     0, sin(alpha(i)), cos(alpha(i)), d(i);
                     0, 0, 0, 1];
        setFixedTransform(jnt, transform);
        
        body.Joint = jnt;
        if i == 1
            addBody(robot, body, robot.BaseName);
        else
            addBody(robot, body, ['body' num2str(i-1)]); 
        end
    end

    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25, 0.25, 0.25, 1, 1, 1];  
    endEffector = robot.BodyNames{end}; 

    targetPosition = [500, 300, 200];  % 목표 위치
    targetOrientation = eul2rotm([0, 0, 0]);  

    tform = trvec2tform(targetPosition) * rotm2tform(targetOrientation);

    figure;
    ax = show(robot);
    title('Robot Moving to Target Position');
    hold on;

    numSteps = 50;
    initialGuess = homeConfiguration(robot);
    for step = 1:numSteps
        intermediatePosition = (1 - step/numSteps) * [0, 0, 0] + (step/numSteps) * targetPosition;
        tformIntermediate = trvec2tform(intermediatePosition) * rotm2tform(targetOrientation);
        
        [configSol, ~] = ik(endEffector, tformIntermediate, weights, initialGuess);

        show(robot, configSol, 'Parent', ax, 'PreservePlot', false);
        drawnow;

        initialGuess = configSol;
    end

    disp('Simulation complete. Robot reached the target position.');
end

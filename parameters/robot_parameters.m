function robot = robot_parameters()
    robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 5);
    
    link_properties = {
        struct('mass', 53.341, 'com', [31.881, 891.094, 398.289]*1e-3, 'inertia', [0.864, 1.092, 1.189, 0, -0.115, 0], 'a', 0, 'd', 270, 'alpha', 0, 'theta_offset', 0),
        struct('mass', 39.345, 'com', [-1.243, 891.094, 680.381]*1e-3, 'inertia', [0.763, 0.835, 0.297, 0, -0.0005751, 0], 'a', 0, 'd', 236, 'alpha', pi/2, 'theta_offset', 0),
        struct('mass', 41.857, 'com', [2.042, 891.094, 1002.596]*1e-3, 'inertia', [2.001, 1.753, 0.43, 0, -0.016, 0], 'a', 500, 'd', 0, 'alpha', 0, 'theta_offset', pi/2),
        struct('mass', 24.017, 'com', [8.833, 911.094, 1525.873]*1e-3, 'inertia', [0.617, 0.62, 0.085, 0, -0.009, 0], 'a', 460, 'd', 0, 'alpha', 0, 'theta_offset', 0),
        struct('mass', 3.195, 'com', [11.461, 911.094, 1787.483]*1e-3, 'inertia', [0.017, 0.018, 0.006, 0, -0.0001314, 0], 'a', 195, 'd', 0, 'alpha', 0, 'theta_offset', 0)
    };
    
    for i = 1:5
        body = rigidBody(['link' num2str(i)]);
        
        if i < 5 
            jnt = rigidBodyJoint(['joint' num2str(i)], 'revolute');
        else
            jnt = rigidBodyJoint(['fixed' num2str(i)], 'fixed'); 
        end

        a = link_properties{i}.a;
        d = link_properties{i}.d;
        alpha = link_properties{i}.alpha;
        theta_offset = link_properties{i}.theta_offset;
        
        transform = [cos(theta_offset), -sin(theta_offset)*cos(alpha), sin(theta_offset)*sin(alpha), a*cos(theta_offset);
                     sin(theta_offset), cos(theta_offset)*cos(alpha), -cos(theta_offset)*sin(alpha), a*sin(theta_offset);
                     0, sin(alpha), cos(alpha), d;
                     0, 0, 0, 1];
        
        setFixedTransform(jnt, transform);
        body.Joint = jnt;

        % 물성치 설정
        body.Mass = link_properties{i}.mass;
        body.CenterOfMass = link_properties{i}.com;
        body.Inertia = link_properties{i}.inertia;  % 6-요소 벡터로 설정
        
        if i == 1
            addBody(robot, body, robot.BaseName);
        else
            addBody(robot, body, ['link' num2str(i-1)]);
        end
    end
end
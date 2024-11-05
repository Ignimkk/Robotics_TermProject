
function DH_table_visualize3
    a = [0, 0, 500, 460, 195];
    alpha = [0, pi/2, 0, 0, 0];
    d = [270, 236, 0, 0, 0];
    theta_offset = [0, 0, pi/2, 0, 0];  % 각 링크의 초기 theta 오프셋 값

    robot = rigidBodyTree;
    for i = 1:length(a)
        body = rigidBody(['body' num2str(i)]);
        jnt = rigidBodyJoint(['joint' num2str(i)], 'revolute');
        setFixedTransform(jnt, ...
            [cos(theta_offset(i)), -sin(theta_offset(i))*cos(alpha(i)), sin(theta_offset(i))*sin(alpha(i)), a(i)*cos(theta_offset(i));
             sin(theta_offset(i)), cos(theta_offset(i))*cos(alpha(i)), -cos(theta_offset(i))*sin(alpha(i)), a(i)*sin(theta_offset(i));
             0, sin(alpha(i)), cos(alpha(i)), d(i);
             0, 0, 0, 1]);
        body.Joint = jnt;
        if i == 1
            addBody(robot, body, robot.BaseName);
        else
            addBody(robot, body, ['body' num2str(i-1)]);
        end
    end

    q = zeros(1, length(a)); 
    config = homeConfiguration(robot);  
    
    % 로봇 시각화
    figure;
    ax = show(robot, config); 
    hold on;

    % 슬라이더 UI 생성
    sliders = gobjects(1, length(a));
    for i = 1:length(a)
        sliders(i) = uicontrol('Style', 'slider', ...
                               'Min', -pi, 'Max', pi, 'Value', 0, ...
                               'Position', [20, 400 - i * 50, 200, 20], ...
                               'Callback', @(src, event) updateRobotPose());
        uicontrol('Style', 'text', ...
                  'Position', [230, 400 - i * 50, 50, 20], ...
                  'String', ['Joint ' num2str(i)]);
    end

    joint_spheres = gobjects(1, length(a));

    function updateRobotPose()
        for j = 1:length(sliders)
            config(j).JointPosition = sliders(j).Value;  
        end
        
        show(robot, config, 'Parent', ax, 'PreservePlot', false);  
        
        for j = 1:4
            if isgraphics(joint_spheres(j))
                delete(joint_spheres(j));
            end
            
            tform = getTransform(robot, config, ['body' num2str(j)]);
            joint_position = tform(1:3, 4); 
            
            joint_spheres(j) = scatter3(joint_position(1), joint_position(2), joint_position(3), 50, 'filled', ...
                                        'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');  % 빨간색 구체로 조인트 표시
        end
    end

    updateRobotPose(); 
end

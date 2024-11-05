clc;
clear all;
clearvars;
clf;

function inverse_kinematic
    % DH 파라미터 설정
    a = [0, 0, 500, 460, 195];
    alpha = [0, pi/2, 0, 0, 0];
    d = [270, 236, 0, 0, 0];
    theta_offset = [0, 0, pi/2, 0, 0];  % 각 링크의 초기 theta 오프셋 값

    % 로봇 모델 생성
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
    
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25 0.25 0.25 1 1 1];  
    endEffector = robot.BodyNames{end};  

    targetPosition = [600; 0; 200];
    targetOrientation = eye(3); )

    tform = trvec2tform(targetPosition') * rotm2tform(targetOrientation);
    
    initialGuess = homeConfiguration(robot);
    [configSol, solInfo] = ik(endEffector, tform, weights, initialGuess);

    figure;
    ax = show(robot, configSol);
    hold on;

    for j = 1:length(configSol)
        tform = getTransform(robot, configSol, ['body' num2str(j)]);
        joint_position = tform(1:3, 4); 
        
        scatter3(joint_position(1), joint_position(2), joint_position(3), 100, 'filled', ...
                 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k'); 
    end

    disp('Inverse Kinematics Joint Angles:');
    for j = 1:length(configSol)
        disp(['Joint ' num2str(j) ': ' num2str(configSol(j).JointPosition)]);
    end
end

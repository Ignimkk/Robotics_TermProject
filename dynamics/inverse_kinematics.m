function jointAngles = inverse_kinematics(robot, targetPos, targetOrientation)
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25, 0.25, 0.25, 1, 1, 1];
    initialGuess = homeConfiguration(robot);
    tform = trvec2tform(targetPos) * rotm2tform(targetOrientation);
    
    % 역기구학 솔루션 계산
    [configSol, ~] = ik(robot.BodyNames{end}, tform, weights, initialGuess);
    
    % configSol을 직접 jointAngles로 할당
    jointAngles = configSol;
end

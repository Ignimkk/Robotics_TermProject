function calculate_torque_to_target(robot, targetPos)
    startAngles = homeConfiguration(robot);
    targetAngles = inverse_kinematics(robot, targetPos, eye(3));
    
    [jointPositions, jointVelocities, jointAccelerations] = path_planning(startAngles, targetAngles, 50);
    
    for k = 1:size(jointPositions, 2)
        tau = equations_of_motion(robot, jointPositions(:, k), jointVelocities(:, k), jointAccelerations(:, k));
        disp(['Step ', num2str(k), ' Required Torques: ', num2str(tau')]);
    end
end

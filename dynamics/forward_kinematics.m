function endEffectorPos = forward_kinematics(jointAngles)
    robot = robot_parameters();  

    jointAngles = jointAngles(:);
    
    endEffectorPos = getTransform(robot, jointAngles, robot.BodyNames{end}, 'base');
end

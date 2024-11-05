function visualize_robot(robot, jointAngles)
    config = homeConfiguration(robot);
    for i = 1:length(jointAngles)
        config(i).JointPosition = jointAngles(i);
    end
    show(robot, config, 'PreservePlot', false);
    drawnow;
end

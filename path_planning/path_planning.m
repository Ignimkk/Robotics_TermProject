function [jointPositions, jointVelocities, jointAccelerations] = path_planning(startAngles, targetAngles, numSteps)
    jointPositions = zeros(length(startAngles), numSteps);
    for i = 1:length(startAngles)
        jointPositions(i, :) = linspace(startAngles(i), targetAngles(i), numSteps);
    end
    jointVelocities = diff(jointPositions, 1, 2) / (1/numSteps);
    jointAccelerations = diff(jointVelocities, 1, 2) / (1/numSteps);
end

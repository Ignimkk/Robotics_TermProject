robot = robot_parameters();

modelName = 'IndependentJointPIDControl';
open_system(new_system(modelName));

set_param(modelName, 'Solver', 'ode45', 'StopTime', '10');

for i = 1:4
    subsystemName = ['Joint' num2str(i) '_Control'];
    add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/' subsystemName]);
    
    add_block('simulink/Continuous/PID Controller', [modelName '/' subsystemName '/PID']);
    set_param([modelName '/' subsystemName '/PID'], 'P', '10', 'I', '1', 'D', '0.1');
    
    add_block('simulink/Commonly Used Blocks/Sum', [modelName '/' subsystemName '/Sum']);
    set_param([modelName '/' subsystemName '/Sum'], 'Inputs', '+-');
    
    gainValue = num2str(robot.Bodies{i}.Mass); 
    add_block('simulink/Commonly Used Blocks/Gain', [modelName '/' subsystemName '/Gain']);
    set_param([modelName '/' subsystemName '/Gain'], 'Gain', gainValue);
    
    add_block('simulink/Continuous/Integrator', [modelName '/' subsystemName '/Integrator']);
    
    add_block('simulink/Sinks/Scope', [modelName '/' subsystemName '/Scope']);
    
    add_line([modelName '/' subsystemName], 'Sum/1', 'PID/1');
    add_line([modelName '/' subsystemName], 'PID/1', 'Gain/1');
    add_line([modelName '/' subsystemName], 'Gain/1', 'Integrator/1');
    add_line([modelName '/' subsystemName], 'Integrator/1', 'Scope/1');
    
    add_block('simulink/Sources/Constant', [modelName '/' subsystemName '/DesiredPosition']);
    add_block('simulink/Commonly Used Blocks/Out1', [modelName '/' subsystemName '/Output']);
    add_line([modelName '/' subsystemName], 'DesiredPosition/1', 'Sum/1');
    add_line([modelName '/' subsystemName], 'Integrator/1', 'Output/1');
end

save_system(modelName);
sim(modelName);

modelName = 'RobotModel';
open_system(new_system(modelName));

set_param(modelName, 'Solver', 'ode45', 'StopTime', '10');

add_block('nesl_utility/Solver Configuration', [modelName '/Solver Configuration']);

add_block('nesl_utility/World Frame', [modelName '/World Frame']);

add_block('nesl_utility/Mechanism Configuration', [modelName '/Mechanism Configuration']);

add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Base Frame']);
add_block('simscape/Frames and Transforms/Rigid Transform', [modelName '/Base Frame/Base joint to base']);
add_block('simscape/Solid/Solid', [modelName '/Base Frame/Base']);

add_block('simscape/Frames and Transforms/Revolute Joint', [modelName '/Joint 1']);
add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Link 1']);
add_block('simscape/Solid/Solid', [modelName '/Link 1/Link 1']);

add_block('simscape/Frames and Transforms/Revolute Joint', [modelName '/Joint 2']);
add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Link 2']);
add_block('simscape/Solid/Solid', [modelName '/Link 2/Link 2']);

add_block('simscape/Frames and Transforms/Revolute Joint', [modelName '/Joint 3']);
add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Link 3']);
add_block('simscape/Solid/Solid', [modelName '/Link 3/Link 3']);

add_block('simscape/Frames and Transforms/Revolute Joint', [modelName '/Joint 4']);
add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/Link 4']);
add_block('simscape/Solid/Solid', [modelName '/Link 4/Link 4']);

add_block('simscape/Frames and Transforms/Weld Joint', [modelName '/Weld Joint']);
add_block('simulink/Commonly Used Blocks/Subsystem', [modelName '/End Effector']);
add_block('simscape/Solid/Solid', [modelName '/End Effector/End Effector']);

add_line(modelName, 'World Frame/1', 'Solver Configuration/1');
add_line(modelName, 'Base Frame/1', 'Joint 1/1');
add_line(modelName, 'Joint 1/1', 'Link 1/1');
add_line(modelName, 'Link 1/1', 'Joint 2/1');
add_line(modelName, 'Joint 2/1', 'Link 2/1');
add_line(modelName, 'Link 2/1', 'Joint 3/1');
add_line(modelName, 'Joint 3/1', 'Link 3/1');
add_line(modelName, 'Link 3/1', 'Joint 4/1');
add_line(modelName, 'Joint 4/1', 'Link 4/1');
add_line(modelName, 'Link 4/1', 'Weld Joint/1');
add_line(modelName, 'Weld Joint/1', 'End Effector/1');

% 모델 저장
save_system(modelName);

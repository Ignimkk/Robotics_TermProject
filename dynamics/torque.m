robot = robot_parameters();

% 초기 각도 및 운동 상태 설정 (모두 0)
initialJointAngles = [0, 0, pi/2, 0]';  % 초기 각도
initialJointVelocities = [0, 0, 0, 0]';  % 초기 속도
initialJointAccelerations = [0, 0, 0, 0]';  % 초기 가속도

% 초기 위치에서의 토크 계산 (토크가 0이 되도록 설정)
tau_initial = equations_of_motion(robot, initialJointAngles, initialJointVelocities, initialJointAccelerations);
disp('Torque at Initial Position [0, 0, 0, 0]:');
disp(tau_initial);

% 목표 각도 및 운동 상태 설정 (임의의 값)
targetJointAngles = [pi/6, pi/4, pi/3, pi/6]';  % 목표 각도
targetJointVelocities = [0.5, 0.3, -0.2, 0.4]';  % 목표 속도
targetJointAccelerations = [0.1, -0.1, 0.2, -0.3]';  % 목표 가속도

% 목표 위치에서의 토크 계산
tau_target = equations_of_motion(robot, targetJointAngles, targetJointVelocities, targetJointAccelerations);
disp('Torque at Target Position');
disp(tau_target);

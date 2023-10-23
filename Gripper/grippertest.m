clc
dobot = DobotCR5();
ur3 = UR3();

hold on

dobot.model.delay = 0;
dobot.model.teach();
drawnow();

ur3.model.base = transl(-2, 0, 0);
ur3.model.delay = 0;
ur3.model.teach();
drawnow();
axis equal

dobotGripper = GripperPlace(dobot.model); % Initialize GripperPlace with the Dobot model
ur3Gripper = GripperPlace(ur3.model);     % Initialize GripperPlace with the UR3 model

steps = 50;
qp = [pi/2, pi/3, 0, 0, 0, -pi/4]; % Define your target joint position

qD = jtraj(dobot.model.getpos, qp, steps); % Joint trajectory for Dobot
qU = jtraj(ur3.model.getpos, qp, steps);   % Joint trajectory for UR3

JointPos = [0, 0, 0];

for i = 1:steps
    dobot.model.animate(qD(i, :)); % Animate the Dobot
    ur3.model.animate(qU(i, :));   % Animate the UR3

    dobotNTB = dobot.model.fkine(qD(i, :)).T; % Calculate Dobot's end-effector transformation
    ur3NTB = ur3.model.fkine(qU(i, :)).T;    % Calculate UR3's end-effector transformation

    animateGripper(dobotGripper, JointPos, dobotNTB); % Animate the Dobot's gripper
    animateGripper(ur3Gripper, JointPos, ur3NTB);     % Animate the UR3's gripper
    
    pause(0.02)
end

tapClaw(dobotGripper)
glassClaw(dobotGripper)
openClaw(dobotGripper)
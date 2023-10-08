% setup robot for the gripper fingers to be placed on the end of the UR3 robot

%structure of code inspired by mdl_baxter robot from robotics toolbox

link(1)=Link('d',0.0,'a',0.00,'alpha',pi/2,'offset',pi,'qlim',deg2rad([0 0]));
link(2)=Link('d',0,'a',0.05,'alpha',0,'offset',deg2rad(10),'qlim',deg2rad([0 60]));
link(3)=Link('d',0,'a',0.03,'alpha',0,'offset',deg2rad(30),'qlim',deg2rad([0 90]));

gripperRight=SerialLink(link,'name','gripperRight');
gripperLeft=SerialLink(link,'name','gripperLeft');

gripperRight.base=transl(0.01,0,0)*trotz(deg2rad(180));
gripperLeft.base=transl(-0.01,0,0);

qClosed=[0 0.7304 1.2789];
qOpen = [0 0 0];



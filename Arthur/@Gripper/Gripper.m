% setup robot for the gripper fingers to be placed on the end of the UR3 robot

%structure of code inspired by mdl_baxter robot from robotics toolbox

link(1)=Link('d',0.0,'a',0.00,'alpha',pi/2,'offset',pi,'qlim',deg2rad([0 0]));
link(2)=Link('d',0,'a',0.05,'alpha',0,'offset',deg2rad(80),'qlim',deg2rad([0 60]));
link(3)=Link('d',0,'a',0.03,'alpha',0,'offset',deg2rad(0),'qlim',deg2rad([0 90]));
link(4)=Link('d',0,'a',0.02,'alpha',0,'offset',deg2rad(10),'qlim',deg2rad([0 30]));


A=SerialLink(link,'name','gripperA');
B=SerialLink(link,'name','gripperB');
C=SerialLink(link,'name','gripperC');

A.base = transl(0.0,  -0.04,   0) * trotz(deg2rad(90));
B.base = transl(0.055,  0.04, 0) * trotz(deg2rad(-90));
C.base = transl(-0.055, 0.04, 0) * trotz(deg2rad(-90));

qClosed=[0 0.7304 1.2789 0];
qOpen = [0 0 0 0];



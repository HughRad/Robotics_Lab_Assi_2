clf
clear
r = UR3;

r = r.model;
x = r.fkine(r.getpos);
x2 = x*transl(0,0,-0.1);
R = zeros(1,3);


a = RMRC(r,[x,x2],R);

q = a.calculateQMatrix;









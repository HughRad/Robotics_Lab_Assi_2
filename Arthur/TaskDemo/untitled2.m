%init
clf

T1 = transl(2,2,2);
han = PlaceObject('hand.ply',[2,2,2]);
[~,v,~] = plyread('hand.ply', 'tri');
pause(5);

pos = [s1,s2,s3];

T2 = transl(pos(1),pos(2),pos(3));

Tr = inv(T1)*T2;

baseVert = [v, ones(size(v, 1), 1)]*T2';
set(han, 'Vertices', baseVert(:, 1:3));
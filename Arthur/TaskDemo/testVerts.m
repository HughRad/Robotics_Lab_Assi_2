clear all
clc

id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

offset = 0.84;  % 0.8400m off ground
[X, Z] = meshgrid(-0.95:0.1:0.95, (-0.15+offset):0.1:(0.15+offset)); % Adjust the ranges to create a 1.9m x 0.3m plane
Y = zeros(size(X)); % Set Z equal to X to create a 2D plane
lightbarrier = surf(X, Y, Z);
hold on
hand = transl(0,-0.6,0.8);
centerPoint = [hand(1,4),hand(2,4),hand(3,4)];
radii = [0.15,0.18,0.1];
han = PlaceObject('hand.ply',[hand(1,4),hand(2,4),hand(3,4)]);
axis equal
% extract vertices
[~,v,~] = plyread('hand.ply', 'tri');
[x,y,z] = ellipsoid(hand(1,4),hand(2,4),hand(3,4),0.15,0.2,0.1);

sgield = surf(x, y, z);
set(sgield, 'EdgeColor', 'none','FaceAlpha', 0);
points = [X(:), Y(:), Z(:)];

%%
%move hand into barrier, ellipsoid collision check

ymove = transl(0,0.01,0);
for i = 1:100
    hand = hand*ymove;
    centerPoint = [hand(1,4),hand(2,4),hand(3,4)];
    [x,y,z] = ellipsoid(centerPoint(1),centerPoint(2),centerPoint(3),...
        radii(1),radii(2),radii(3));
    baseVert = [v, ones(size(v, 1), 1)]*hand';
    set(han, 'Vertices', baseVert(:, 1:3));
    sgield = surf(x,y,z);
    algebraicDist = ((points(:,1) - centerPoint(1)) / radii(1)).^2 ...
              + ((points(:,2) - centerPoint(2)) / radii(2)).^2 ...
              + ((points(:,3) - centerPoint(3)) / radii(3)).^2;
    pointsInside = find(algebraicDist < 1);
    if size(pointsInside, 1) > 1
        disp('oh noh');
        break
    else
    set(sgield, 'EdgeColor', 'none','FaceAlpha', 0);
    end
    pause(0.02)
end

set(sgield, 'EdgeColor', 'none','FaceAlpha', 0);
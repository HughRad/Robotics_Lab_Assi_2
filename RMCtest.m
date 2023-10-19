%%Resolved motion Control Test
clc
clf

U = UR3(); 
UR = U.model; 
% UR.delay = 0;

%%start/settings
t = 10;             % Total sim time (s)
deltaT = 0.02;      % Control frequency (time between steps)
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares (how close to sing before it modifyies path)
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector (emphasis on tans or rot velocity)


%Pre-Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

base = UR.base().T;

% Set up trajectory,
waypoints = [0.3,-0.3,0.2;
            0.3,0.3,0.4;
            -0.2,0.4,0.4;]';
s = trapveltraj(waypoints,steps);     % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = s(1,i); % Points in x
    x(2,i) = s(2,i); % Points in y
    x(3,i) = s(3,i); % Points in z
    theta(1,i) = -(atan2((x(2,i)-base(2,4)),(x(1,i)-base(1,4))));    % Roll angle 
    theta(2,i) = pi/2;            % Pitch angle
    theta(3,i) = 0;        % Yaw angle
end

hold on
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)

%first posision of robot
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];  % Create transformation of first point and angle
q0 = [2.1363,-0.6283,1.6336,-2.3876,1.5080,0];                        % Initial guess for joint angles
qMatrix(1,:) = UR.ikcon(T,q0); 

% Track the trajectory with RMRC
for i = 1:steps-1
    T = UR.fkine(qMatrix(i,:)).T; % Get forward transformation at current joint state(to ensure accuracy)
    deltaX = x(:,i+1) - T(1:3,4);  % Get position error from next waypoint (diff between next location and current xzy position)
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);      % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);        % Calculate rotation matrix error (diff between nex angle and current)

    S = Rdot*Ra';         % Skew symmetric matrix contains the angular velocities
    linear_velocity = (1/deltaT)*deltaX; %velocit to next point
    angular_velocity = [S(3,2);S(1,3);S(2,1)]; %Velocity to next angle, extracted from Skew Symmetric matrix
    deltaTheta = tr2rpy(Rd*Ra');     % Convert rotation matrix/error to RPY angles
    xdot = W*[linear_velocity;angular_velocity];   % Calculate end-effector velocity to reach next waypoint. * by our weights

    J = UR.jacob0(qMatrix(i,:));     % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J')); %get end effector manipuability

    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2; %(*a value for lambda max)
    else %does DLS if the arm aproches singularity
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J'; % Damped Least Squared Inverse

    qdot(i,:) = (invJ*xdot)';  % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6           % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < UR.qlim(j,1)  % If next joint angle is lower than joint limit...(if velocities will cause joint angles to go below the limit)
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > UR.qlim(j,2)  % If next joint angle is greater than joint limit ... (if velocities will cause joint angles to go above the limit)
            qdot(i,j) = 0; % Stop the motor
        end
    end

    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);     % Update next joint state based on joint velocities
end


%Plot the results
for i = 1:1:steps 
    UR.animate(qMatrix(i,:)); 
    pause(0.01);
end



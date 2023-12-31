classdef RMCDemo2 < handle

    properties
        % Robots
        CO; 
        UR;
        URgrip;  %grippers
        COgrip;
        gripJ = [0,0]; %gripper static pos

        % Environment
        glasspos;  %starting glass locations
        glassPLY;  %glass model info
        cupSpot;   %cup pouring destination locations (beer taps 1-4)

        % Functionality
        glass; %the target glass
        beer; %the target beer
    end

    methods
       function self = RMCDemo2()
            clc
            clf

            U = UR3(transl([0.30,0.66,0.68]));
            hold on;

            self.UR = U.model;
            
            self.UR.delay = 0;
            self.UR.animate([0,0,0,-pi/2,0,0]); 
    
            % load in the environment
            self.createEnvironment;

            axis([-1,1,0,1.5,0.5,1.5]); %close up
            % axis([-2,2,-2,2,0,2]); %full View
            view(3);

            % initialise target glass and beer
            self.glass = 1;
            self.beer = 1;
            disp("Select a beer from the GUI");
            input('hit enter')

            %test
            self.serveBeer;

       end

       function serveBeer(self) 
           for i = 0.1:0.1:5
               nextpos = [i/10,0.5,0.7];
               self.BeerServer(nextpos) %Beer serving animation
           end
       end

       function Ranimate(self,URqMatrix) %robot animation function

            for i = 1:1:size(URqMatrix)
                self.UR.animate(URqMatrix(i,:)); 

                pause(0.001);
            end
        end

        function BeerServer(self,nextpos) %UR bot cup filler/placer

           %to cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,0,0; 0,0,0;]';
           waypoints = [current; nextpos;]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(URqMatrix);


       end

       function self = createEnvironment(self)


           self.glasspos = {[0.03,1.1,0.71],[0.16,1.1,0.71]...
               ,[0.29,1.1,0.71],[0.42,1.1,0.71]};

           self.glassPLY = [(PlaceObject('Glass.ply',self.glasspos{1,1}))...  %placing cups at starting location
               ,(PlaceObject('Glass.ply',self.glasspos{1,2}))...
               ,(PlaceObject('Glass.ply',self.glasspos{1,3}))...
               ,(PlaceObject('Glass.ply',self.glasspos{1,4}))];

           self.cupSpot = {[-0.02,0.26,0.7],[0.13,0.26,0.7],[0.28,0.26,0.7]...
               ,[0.43,0.26,0.7],[0.73,0.20,0.7],[-0.68,0.20,0.7]...
               ,[-0.605,1.205,0.7]};
       end
    end

    methods (Static)
        function qMatrix = RMCgen(robot,waypoints,angles) %RMC path gen
            %%start/settings
            t = 0.01;             % Total sim time (s)
            deltaT = 0.002;      % Control frequency (time between steps)
            steps = t/deltaT;   % No. of steps for simulation
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares (how close to sing before it modifyies path)
            W = diag([1 1 1 1 1 1]);    % Weighting matrix for the velocity vector (emphasis on tans or rot velocity)

            %Pre-Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory

            % Set up trajectory,
            s = trapveltraj(waypoints,steps);     % Trapezoidal trajectory scalar
            a = trapveltraj(angles,steps);
            for i=1:steps
                x(1,i) = s(1,i); % Points in x
                x(2,i) = s(2,i); % Points in y
                x(3,i) = s(3,i); % Points in z
                theta(1,i) = a(1,i);    % Roll angle
                theta(2,i) = a(2,i);    % Pitch angle
                theta(3,i) = a(3,i);    % Yaw angle
            end

            hold on
            plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)

            %first position of robot
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];  % Create transformation of first point and angle
            q0 = [robot.getpos];          % Initial guess for joint angles
            qMatrix(1,:) = robot.ikcon(T,q0);

            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:)).T; % Get forward transformation at current joint state(to ensure accuracy)
                deltaX = x(:,i+1) - T(1:3,4);  % Get position error from next waypoint (diff between next location and current xzy position)
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);      % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);        % Calculate rotation matrix error (diff between nex angle and current)

                S = Rdot*Ra';         % Skew symmetric matrix contains the angular velocities
                linear_velocity = (1/deltaT)*deltaX; %velocit to next point
                angular_velocity = [S(3,2);S(1,3);S(2,1)]; %Velocity to next angle, extracted from Skew Symmetric matrix
                xdot = W*[linear_velocity;angular_velocity];   % Calculate end-effector velocity to reach next waypoint. * by our weights

                J = robot.jacob0(qMatrix(i,:));     % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J')); %get end effector manipuability

                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2; %(*a value for lambda max)
                else %does DLS if the arm aproches singularity
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J'; % Damped Least Squared Inverse

                qdot(i,:) = (invJ*xdot)';  % Solve the RMRC equation (you may need to transpose the vector)
                for j = 1:6           % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)  % If next joint angle is lower than joint limit...(if velocities will cause joint angles to go below the limit)
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)  % If next joint angle is greater than joint limit ... (if velocities will cause joint angles to go above the limit)
                        qdot(i,j) = 0; % Stop the motor
                    end
                end

                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);     % Update next joint state based on joint velocities
            end
        end
    end
end
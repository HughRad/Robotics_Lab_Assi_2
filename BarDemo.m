classdef BarDemo

    % properties
    %     Property1
    % end

    methods
       function self = BarDemo()
            clc
            clf

            C = DobotCR5(transl([-0.25,0.66,0.68])); %Load bots (with offset)
            hold on;
            U = UR3(transl([0.30,0.66,0.68])); 

            CO = C.model; %account for .model
            UR = U.model; 

            CO.delay = 0; %remove delay + set intial pos
            UR.delay = 0;
            UR.animate([0,-pi/2,0,-pi/2,0,0]);
            
            PlaceObject('BarBase.ply',[0,0,0]); %load bar models
            PlaceObject('BarSafe.ply',[0,0,0]);
            PlaceObject('BeerTap.ply',[-0.05,0,0]);
            PlaceObject('BeerTap.ply',[0.10,0,0]);
            PlaceObject('BeerTap.ply',[0.25,0,0]);
            PlaceObject('BeerTap.ply',[0.40,0,0]);

            Glasspos = {[0.03,1.1,0.71],[0.16,1.1,0.71]...
                        ,[0.29,1.1,0.71],[0.42,1.1,0.71]};
            GlassPLY = [(PlaceObject('Glass.ply',Glasspos{1,1}))...  %placing cups at starting location
                       ,(PlaceObject('Glass.ply',Glasspos{1,2}))...  
                       ,(PlaceObject('Glass.ply',Glasspos{1,3}))...  
                       ,(PlaceObject('Glass.ply',Glasspos{1,4}))];

            CupSpot = {[-0.02,0.26,0.7],[0.13,0.26,0.7],[0.28,0.26,0.7]...
                        ,[0.43,0.26,0.7],[0.73,0.20,0.7]};
            temply = [(PlaceObject('Glass.ply',CupSpot{1,1}))...  %placing cups at starting location
                       ,(PlaceObject('Glass.ply',CupSpot{1,2}))...  
                       ,(PlaceObject('Glass.ply',CupSpot{1,3}))...
                       ,(PlaceObject('Glass.ply',CupSpot{1,4}))...  
                       ,(PlaceObject('Glass.ply',CupSpot{1,5}))];

            [f1,v1,data1] = plyread('Robotiq_3finger.ply','tri'); %gets gripper vertex info at 0,0,0
            COgrip = PlaceObject('Robotiq_3finger.ply',[0,0,0]); %load gripper model
            URgrip = PlaceObject('Robotiq_3finger.ply',[0,0,0]); 

            gripvertfin = CO.fkine(CO.getpos).T*trotz(pi/2)*transl(0,0,-0.003); 
            transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
            set(COgrip,'Vertices',transVertfin(:,1:3)); 

            gripvertfin = UR.fkine(UR.getpos).T*trotz(pi/2)*transl(0,0,-0.003); 
            transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
            set(URgrip,'Vertices',transVertfin(:,1:3)); 

            axis([-1,1,0,1.5,0.5,1.5]); %close up
            % axis([-2,2,-2,2,0,2]); %full View
            view(3);

            % UR.teach([0,-pi/2,0,-pi/2,0,0]);

            glass = 1; %target glass

            while true
                disp("Which beer would you like?:");
                op = input("1.Beer, 2.Beer, 3.Beer or 4.Beer (enter 5 to exit program)...");
    
                switch op %remember to hit 5 to exit the program
                    case 1
                        disp("you selected 1");
                        GPick = Glasspos{glass} + [0,-0.1,0.05]; %mod to grab glass right
                        GPlace = CupSpot{1} + [0,0.1,0.05];

                        self.BeerServer(UR,GPick,GPlace,URgrip)

                        glass = glass + 1; %cycle through glasses
                        if glass == 5
                            glass = 1;
                        end

                    case 2 
                        disp("you selected 2");
                        GPick = Glasspos{glass} + [0,-0.1,0.05]; 
                        GPlace = CupSpot{2} + [0,0.1,0.05];

                        self.BeerServer(UR,GPick,GPlace,URgrip)

                        glass = glass + 1; 
                        if glass == 5
                            glass = 1;
                        end

                    case 3
                        disp("you selected 3");
                        GPick = Glasspos{glass} + [0,-0.1,0.05]; 
                        GPlace = CupSpot{3} + [0,0.1,0.05];

                        self.BeerServer(UR,GPick,GPlace,URgrip)

                        glass = glass + 1; 
                        if glass == 5
                            glass = 1;
                        end

                    case 4
                        disp("you selected 4");
                        GPick = Glasspos{glass} + [0,-0.1,0.05]; 
                        GPlace = CupSpot{4} + [0,0.1,0.05];

                        self.BeerServer(UR,GPick,GPlace,URgrip)

                        glass = glass + 1; 
                        if glass == 5
                            glass = 1;
                        end

                    case 5
                        disp("Exiting");
                        break;
                    otherwise
                        disp("That was not an option!");
                end
            end
       end
    end

    methods (Static)
        function qMatrix = RMCgen(robot,waypoints,angles) %RMC path gen
            %%start/settings
            t = 3;             % Total sim time (s)
            deltaT = 0.02;      % Control frequency (time between steps)
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
            
            % hold on
            % plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            
            %first posision of robot
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
        
        function Ranimate(robot,qMatrix,grip) %robot animation function
            [f1,v1,data1] = plyread('Robotiq_3finger.ply','tri'); %gets gripper vertex info at 0,0,0

            for i = 1:1:size(qMatrix)
                robot.animate(qMatrix(i,:)); 

                gripvertfin = robot.fkine(qMatrix(i,:)).T*trotz(pi/2)*transl(0,0,-0.003); 
                transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                set(grip,'Vertices',transVertfin(:,1:3)); 

                pause(0.01);
            end
        end

        function BeerServer(UR,GPick,GPlace,URgrip) %UR bot cup filler/placer
            %to start
            q0 = [-1.6336,-1.1938,1.5080,-0.3142,1.5080,0];
            tr = transl(GPick+[0,0,0.2])*troty(pi/2)*trotx(-pi/2);
            newQ = UR.ikcon(tr,q0);
            qMatrix = jtraj(UR.getpos,newQ,50);
            BarDemo.Ranimate(UR,qMatrix,URgrip);

            %to cup
            current = tform2trvec(UR.fkine(UR.getpos).T);
            angles = [-pi/2,pi/2,0; -pi/2,pi/2,0;]';
            waypoints = [current; GPick;]';
            qMatrix = BarDemo.RMCgen(UR,waypoints,angles);        
            BarDemo.Ranimate(UR,qMatrix,URgrip);

            pause(3);
            
            %to tap
            current = tform2trvec(UR.fkine(UR.getpos).T);
            angles = [-pi/2,pi/2,0; -pi/2,pi/2,0; -pi/2,pi/2,0;
                      -pi,pi/2,0; (-270*pi/180),pi/2,0;]';
            waypoints = [current; (GPick+[0,0,0.2]);0,0.9,0.9;
                        -0.05,0.5,0.8; GPlace;]';
            qMatrix = BarDemo.RMCgen(UR,waypoints,angles);        
            BarDemo.Ranimate(UR,qMatrix,URgrip);

            pause(3);

            %to end
            current = tform2trvec(UR.fkine(UR.getpos).T);
            angles = [(-270*pi/180),pi/2,0; (-270*pi/180),pi/2,0;
                      (-310*pi/180),pi/2,0;]';
            waypoints = [current; 0.50,0.40,0.8;
                        0.66,0.30,0.75]';
            qMatrix = BarDemo.RMCgen(UR,waypoints,angles);        
            BarDemo.Ranimate(UR,qMatrix,URgrip);

            pause(3);

            %retract
            current = tform2trvec(UR.fkine(UR.getpos).T);
            angles = [(-310*pi/180),pi/2,0; (-310*pi/180),pi/2,0;]';
            waypoints = [current; 0.50,0.40,0.8;
                        0.7,0.6,1]';
            qMatrix = BarDemo.RMCgen(UR,waypoints,angles);        
            BarDemo.Ranimate(UR,qMatrix,URgrip);

            q0 = [0,-pi/2,0,-pi/2,0,0];
            qMatrix = jtraj(UR.getpos,q0,50);
            BarDemo.Ranimate(UR,qMatrix,URgrip);
        end

    end
end
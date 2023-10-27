classdef BarDemo < handle

    properties
        
        % App
        pendant;


        % Robots
        C;
        U;
        CO;
        UR;
        URgrip;
        COgrip;


        % Environment
        glasspos;
        glassPLY;
        cupSpot;

        % Functionality
        glass; %!< INT The glass to target
        beer; %!< INT The selected beer
    end

    methods
       function self = BarDemo(app)
            clf

            self.pendant = app;

            self.C = DobotCR5(transl([-0.60,0.70,0.68])); %Load bots (with offset)
            hold on;
            self.U = UR3(transl([0.30,0.66,0.68]));

            self.CO = self.C.model; %account for .model
            self.UR = self.U.model;

            self.CO.delay = 0; %remove delay + set intial pos
            self.UR.delay = 0;
            self.UR.animate([0,-pi/2,0,-pi/2,0,0]);

            [f1,v1,data1] = plyread('Robotiq_3finger.ply','tri'); %gets gripper vertex info at 0,0,0
            self.COgrip = PlaceObject('Robotiq_3finger.ply',[0,0,0]); %load gripper model
            self.URgrip = PlaceObject('Robotiq_3finger.ply',[0,0,0]);
    
            % load in the environment
            self.createEnvironment;

            gripvertfin = self.CO.fkine(self.CO.getpos).T*trotz(pi/2)*transl(0,0,-0.003);
            transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin';
            set(self.COgrip,'Vertices',transVertfin(:,1:3));

            gripvertfin = self.UR.fkine(self.UR.getpos).T*trotz(pi/2)*transl(0,0,-0.003);
            transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin';
            set(self.URgrip,'Vertices',transVertfin(:,1:3));

            axis([-1,1,0,1.5,0.5,1.5]); %close up
            % axis([-2,2,-2,2,0,2]); %full View
            view(3);

            % initialise target glass and beer
            self.glass = 1;
            self.beer = 1;
            disp("Select a beer from the GUI");


       end

       function serveBeer(self)

           
           GPick = self.glasspos{self.glass} + [0,-0.1,0.05]; %mod to grab glass right
           GPlace = self.cupSpot{self.beer} + [0,0.1,0.05];
           GP = self.glassPLY(self.glass); %Glass PLY to move

           self.BeerServer(GPick,GPlace,GP)

           Pickpos = self.GlassReturn(GP);
           GPick2 = self.cupSpot{7} + [0,-0.1,0.05];
           GPlace2 = Pickpos + [0,0.1,0.05];

           self.BeerRecaller(GPick2,GPlace2,GP)

           self.BeerReturner(GPick,GPlace,GP)

           self.glass = self.glass + 1; %cycle through glasses
           if self.glass == 5
               self.glass = 1;
           end



       end

       function Ranimate(self,robot,qMatrix,grip,GP,robot2,grip2,qMatrix2) %robot animation function
           
            [f1,v1,data1] = plyread('Robotiq_3finger.ply','tri'); %gets gripper vertex info at 0,0,0
            [f,v,data] = plyread('Glass.ply','tri'); %gets vector position data for glass ply at origin

            for i = 1:1:size(qMatrix)
                robot.animate(qMatrix(i,:)); 

                gripvertfin = robot.fkine(qMatrix(i,:)).T*trotz(pi/2)*transl(0,0,-0.003); 
                transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                set(grip,'Vertices',transVertfin(:,1:3)); 

                if exist('GP', 'var') == 1
                    grabvert = robot.fkine(qMatrix(i,:)).T*troty(-pi/2)*transl(0.1,0,-0.05); %gets current UR3 end effector position
                    transVert = [v,ones(size(v,1),1)]*grabvert'; %gets new values for the brick vertice positions
                    set(GP,'Vertices',transVert(:,1:3)); %sets current brick vertices to new pos
                end

                if exist('robot2', 'var') == 1
                    robot2.animate(qMatrix2(i,:));

                    gripvertfin = robot2.fkine(qMatrix2(i,:)).T*trotz(pi/2)*transl(0,0,-0.003); 
                    transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                    set(grip2,'Vertices',transVertfin(:,1:3));
                end

                self.pendant.actionEStopStatus;

                pause(0.01);
            end
       end

       function emergencyStop(self)


       end



       function BeerServer(self,GPick,GPlace,GP) %UR bot cup filler/placer

           %to start
           q0 = [-1.6336,-1.1938,1.5080,-0.3142,1.5080,0];
           tr = transl(GPick+[0,0,0.2])*troty(pi/2)*trotx(-pi/2);
           newQ = self.UR.ikcon(tr,q0);
           qMatrix = jtraj(self.UR.getpos,newQ,50);
           self.Ranimate(self.UR,qMatrix,self.URgrip);

           %to cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0;]';
           waypoints = [current; GPick;]';
           qMatrix = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(self.UR,qMatrix,self.URgrip);

           pause(3);

           %to tap
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0; -pi/2,pi/2,0;
               -pi,pi/2,0; (-270*pi/180),pi/2,0;]';
           waypoints = [current; (GPick+[0,0,0.2]);0,0.9,0.9;
               -0.05,0.5,0.8; GPlace;]';
           qMatrix = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(self.UR,qMatrix,self.URgrip,GP);

           pause(3);

           %to end
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-270*pi/180),pi/2,0; (-270*pi/180),pi/2,0;
               (-310*pi/180),pi/2,0;]';
           waypoints = [current; 0.60,0.40,0.8;
               0.66,0.30,0.75]';
           qMatrix = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(self.UR,qMatrix,self.URgrip,GP);

           pause(3);

           %retract
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-310*pi/180),pi/2,0; (-310*pi/180),pi/2,0;
               (-2*pi),pi/2,0]';
           waypoints = [current; 0.60,0.40,0.8;
               0.75,0.66,1]';
           qMatrix = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(self.UR,qMatrix,self.URgrip);

           q0 = [0,-pi/2,0,-pi/2,0,0];
           qMatrix = jtraj(self.UR.getpos,q0,50);
           self.Ranimate(self.UR,qMatrix,self.URgrip);
       end

       function BeerRecaller(self,GPick,GPlace,GP)
           %to start
           q0 = [-1.3823,-0.1257,-1.8396,-1.1310,-1.3823,0];
           tr = transl(GPlace+[0,0,0.2])*troty(pi/2)*trotx(pi/2);
           newQ = self.CO.ikcon(tr,q0);
           qMatrix = jtraj(self.CO.getpos,newQ,50);
           self.Ranimate(self.CO,qMatrix,self.COgrip);

           %to dirty cup
           current = tform2trvec(robot.fkine(self.CO.getpos).T);
           angles = [pi/2,pi/2,0; pi/2,pi/2,0;]';
           waypoints = [current; GPlace;]';
           qMatrix = self.RMCgen(self.CO,waypoints,angles);
           self.Ranimate(self.CO,qMatrix,grip);

           pause(3);

           %to washer
           current = tform2trvec(robot.fkine(self.CO.getpos).T);
           angles = [pi/2,pi/2,0; pi/2,-pi/2,0; pi,-pi/2,0;
               3*pi/2,-pi/2,0; 3*pi/2,-pi/2,0;
               3*pi/2,-pi/2,0;3*pi/2,-pi/2,0;]';
           waypoints = [current; (GPlace+[0,0,0.2]);-0.2,0.65,0.9;
               GPick+[0,0,0.2]; (GPick+[0,0,0.08]);
               (GPick+[0,0,0.08]);GPick+[0,0,0.2];]';
           qMatrix = self.RMCgen(self.CO,waypoints,angles);
           self.Ranimate(self.CO,qMatrix,self.COgrip,GP);

           pause(3);

           %handover pt 1 (prepare)
           rendezvous = [-0.15,0.66,0.9];

           q0 = [-0.1257,-0.2513,-1.4067,-1.5080,-1.7593,1.5080];
           tr = transl(rendezvous+[-0.2,0,0])*troty(-pi/2)*trotx(pi);
           newQ = self.CO.ikcon(tr,q0);
           qMatrix = jtraj(self.CO.getpos,newQ,50);

           q0 = [0,-1.3195,1.8850,-0.5655,1.5708,1.5708];
           tr = transl(rendezvous+[0.2,0,0])*troty(-pi/2);
           newQ = selfUR.ikcon(tr,q0);
           qMatrix2 = jtraj(self.UR.getpos,newQ,50);

           self.Ranimate(self.CO,qMatrix,self.COgrip,GP,self.UR,self.URgrip,qMatrix2);

           %handover pt 2 (join)
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi,-pi/2,0; pi,-pi/2,0;]';
           waypoints = [current; (rendezvous+[-0.1,0,0.05]);]';
           qMatrix = self.RMCgen(self.CO,waypoints,angles);

           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0;]';
           waypoints = [current; (rendezvous+[0.1,0,0.05]);]';
           qMatrix2 = self.RMCgen(self.UR,waypoints,angles);

           self.Ranimate(self.CO,qMatrix,self.COgrip,GP,self.UR,self.URgrip,qMatrix2);

           pause(3);

           %handover pt 3 (retract)
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi,-pi/2,0; pi,-pi/2,0;]';
           waypoints = [current; (rendezvous+[-0.2,0,0.05]);]';
           qMatrix = self.RMCgen(self.CO,waypoints,angles);

           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0;]';
           waypoints = [current; (rendezvous+[0.2,0,0.05]);]';
           qMatrix2 = self.RMCgen(self.UR,waypoints,angles);

           self.Ranimate(self.UR,qMatrix2,self.URgrip,GP,self.CO,self.COgrip,qMatrix);

           pause(3);

       end

       function BeerReturner(self,GPick,GPlace,GP)
           %return cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0; -pi/2,pi/2,0;-pi/2,pi/2,0;]';
           waypoints = [current; current;(GPick+[0,0,0.2]);GPick;]';
           qMatrix2 = self.RMCgen(self.UR,waypoints,angles);

           self.Ranimate(self.UR,qMatrix2,self.URgrip,GP);

           pause(3);

           %cup retract
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0; -pi/2,pi/2,0;]';
           waypoints = [current; (GPick+[0,0,0.2]);0.3,0.9,0.9;]';
           qMatrix2 = self.RMCgen(self.UR,waypoints,angles);
           self.Ranimate(self.UR,qMatrix2,self.URgrip);

           %return to start
           q0 = [0,0,0,0,0,0];
           qMatrix = jtraj(self.CO.getpos,q0,50);

           q0 = [0,-pi/2,0,-pi/2,0,0];
           qMatrix2 = jtraj(self.UR.getpos,q0,50);

           self.Ranimate(self.CO,qMatrix,self.COgrip,[],self.UR,self.URgrip,qMatrix2)

       end

       function self = createEnvironment(self)

           PlaceObject('BarBase.ply',[0,0,0]); %load bar models
           PlaceObject('BarSafe.ply',[0,0,0]);
           PlaceObject('BeerTap.ply',[-0.05,0,0]);
           PlaceObject('BeerTap.ply',[0.10,0,0]);
           PlaceObject('BeerTap.ply',[0.25,0,0]);
           PlaceObject('BeerTap.ply',[0.40,0,0]);

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
        
       function chooseBeer(self,beer)
           self.beer = beer;

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



        function Pickpos = GlassReturn(GP)
            [f,v,data] = plyread('Glass.ply','tri');

            randomx = -0.5 + (-0.87 + 0.5)*rand;
            randomy = 0.15 + (0.30 - 0.15)*rand;
            Pickpos = [randomx,randomy,0.7];

            transVert = v+Pickpos;
            set(GP,'Vertices',transVert(:,1:3));
        end

    end
end
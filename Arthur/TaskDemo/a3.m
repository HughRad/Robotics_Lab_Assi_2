classdef a3 < handle

    properties (Access = public)
        % Robots
        C;
        U;
        CO; 
        UR;
        URgrip;  %grippers
        COgrip;
        URpos; %current gripper q positions
        COpos;

    end

    properties

        % Environment
        glasspos;  %starting glass locations
        glassPLY;  %glass model info
        cupSpot;   %cup pouring destination locations (beer taps 1-4)
        CupMod;    %Glass to end effector offset (so its grabbed correctly)
        v;         %vector position data for glass ply at origin

        % Obstacle
        hue; %human model for collision test
        
        % Glass/beer Functionality
        glass; %the target glass
        beer; %the target beer
        GlassLock; %if glass is grabbed or not
        GP; %which glass is held
        GR; %which robot is holding the glass

        %Collision Functionality
        trapvert;   %vertex data to be considered in collision detection
        EllipData; %Ellipsoid class to detect collisions
        CollCheck; %true/fase for if a collision happens

        % App
        pendant;

    end

    methods
       function self = a3(app)
            clc
            clf

            if nargin > 0 
                self.pendant = app;
            end

            self.C = DobotCR5(transl([-0.60,0.70,0.68])); %Load bots (with offset)
            hold on;
            self.U = UR3(transl([0.25,0.66,0.68]));

            self.CO = self.C.model; %account for .model
            self.UR = self.U.model;
            
            self.CO.delay = 0; %remove delay + set intial pos
            self.UR.delay = 0;
            self.UR.animate([0,-pi/2,0,-pi/2,0,0]);

            self.COgrip = Place2F85(self.CO); %load gripper model
            self.URgrip = Place2F85(self.UR);

            COend = self.CO.fkine(self.CO.getpos).T; % Calculate end-effector transformations
            URend = self.UR.fkine(self.UR.getpos).T;

            self.COpos = self.COgrip.openClawPos; %sets current claw positions to open
            self.URpos = self.URgrip.openClawPos;

            animateGripper(self.COgrip,self.COpos,COend); % move grippers to start pos (end effectors)
            animateGripper(self.URgrip,self.URpos,URend);   

            % load in the environment
            self.createEnvironment;

            axis([-1,1,0,1.5,0.5,1.5]); %close up
            % axis([-2,2,-2,2,0,2]); %full View
            view(3);

            self.CollCheck = false; %set to assume no collisions are occurring at start

            % initialise target glass and beer
            self.glass = 1;
            self.beer = 1;
            self.GlassLock = false; %glass is not being moved at start
            disp("Select a beer from the GUI");
       end

       function CollisionDemo(self) %loads a human model ply and adds it into collision consideration
           self.hue = PlaceObject('Human.ply',[-0.1,0.5,0.5]);
           self.trapvert = vertcat(self.trapvert,get(self.hue,'Vertices')); %adds humans verticies to collision check consideration
           self.EllipData = Ellipsoid(self.UR,self.CO,self.trapvert); %reloads the object with the new values
       end

       function CollisionDemoEnd(self) %removes the human ply and removes it from collision consideration
           self.trapvert = self.trapvert(1:size(get(self.hue,'Vertices'),1),:); %removes humans verticies to collision check consideration
           delete(self.hue); %delete model
           self.EllipData = Ellipsoid(self.UR,self.CO,self.trapvert); %reloads the object with the new values
       end

       function oneStep(self,robot,Tstep)

           % get the current position of the robot
           T1 = robot.fkine(robot.getpos).T;

           % get the desired end effector position
           Tr = Tstep;
           T2 = T1 * Tr;

           % use RMRC to move to position
           waypoints = [tform2trvec(T1); tform2trvec(T2)]';
           angles = [tr2rpy(T1); tr2rpy(T2)]';
           t = 0.001;
           qMatrix = self.RMCgen(robot,waypoints,angles,t);

           % use Ranimate to animate the movement
           if robot == self.UR
               gripper = self.URgrip;
               gripPos = self.URpos;
           elseif robot == self.CO
               gripper = self.COgrip;
               gripPos = self.COpos;
           end
           
           self.Ranimate(robot,gripper,qMatrix,gripPos);
           

       end

       function serveBeer(self)
           
           GPick = self.glasspos{self.glass} + [0,-0.125,0.07]; %cup Grabbing location + offset to grab glass right 
           GPlace = self.cupSpot{self.beer} + [0,0.125,0.07];   %Cup filling location + offset to grab glass right
         
           % self.CollisionDemo %for testing

           self.BeerServer(GPick,GPlace) %Beer serving animation cycle
           % self.GP = self.glassPLY(self.glass); %for testing

           Pickpos = self.GlassReturn(); %moves glass to collection bay and gives its new pick up pos
           
           GPick2 = self.cupSpot{7} + [0,-0.125,0.07]; %glass cleaning area location
           GPlace2 = Pickpos + [0,0.125,0.07]; %glass pick up location (names are not accurate)

           self.BeerRecaller(GPick2,GPlace2) %Beer cleaning + handover animation cycle

           self.BeerReturner(GPick) %Cup storing and return animation cycle

           self.glass = self.glass + 1; %cycle through glasses for next run
           if self.glass == 5
               self.glass = 1;
           end

       end

       function Ranimate(self,Robot,Gripper,qMatrix,qGrip) %robot animation Loop function
           for i = 1:1:size(qMatrix)
                
               % check the status of the estop
               pause(0.05);
               self.pendant.actionEStopStatus([self.UR.getpos; self.CO.getpos]); 
               

               % animate object movement if it exists
               if self.pendant.TogglehandButton.Value == true

                   self.pendant.animateHand;

                   % check if the hand is inside the light curtain
                   % pause the animation until the hand is removed

                   while self.pendant.checkLightCurtain
                       self.pendant.animateHand;
                       self.pendant.actionEStopStatus([self.UR.getpos; self.CO.getpos]);
                       pause(0.001);
                   end

               end

               Robot.animate(qMatrix(i,:)); %moves robot to next q pos

               endeffect = Robot.fkine(Robot.getpos).T; %get end effector pos
               animateGripper(Gripper,qGrip,endeffect); %moves gripper to end effector

               if Robot == self.UR && self.pendant.getColButStatus %check for collisions with UR (if toggled)
                   self.CollCheck = self.EllipData.URmesh(qMatrix(i,:)); %runs function to do elipsoid check
               end

               if Robot == self.CO && self.pendant.getColButStatus %check for collisions with CO (if toggled)
                   self.CollCheck = self.EllipData.CR5mesh(qMatrix(i,:)); %runs function to do elipsoid check
               end

               if self.CollCheck == true % if collision came back true, run estop
                   self.pendant.actionCollisionDetection([self.UR.getpos; self.CO.getpos]);
                   self.CollCheck = false; %set col back to false after (assumes collision was fixed) 
               end

               if self.GlassLock == true %animates cup with end effect if glass lock is set to true
                   grabvert = self.GR.fkine(qMatrix(i,:)).T*self.CupMod;
                   transVert = [self.v,ones(size(self.v,1),1)]*grabvert';
                   set(self.GP,'Vertices',transVert(:,1:3));
               end

               self.pendant.updateJointSliders; %update joint sliders
               
           end     
       end

       function chooseBeer(self,beer)

           self.beer = beer; %sets beer choice

       end

       function RanimateDual(self,URqMatrix,COqMatrix,URqGrip,COqGrip) %double robot animation loop (same as ranimate but takes 2 bots) 
           for i = 1:1:size(URqMatrix)
                
               % check the status of the estop
               pause(0.05);
               self.pendant.actionEStopStatus([self.UR.getpos; self.CO.getpos]);

               % animate object movement if it exists
               if self.pendant.TogglehandButton.Value == true

                   self.pendant.animateHand;

                   % check if the hand is inside the light curtain
                   % pause the animation until the hand is removed

                   while self.pendant.checkLightCurtain
                       self.pendant.animateHand;
                       self.pendant.actionEStopStatus([self.UR.getpos; self.CO.getpos]);
                       pause(0.001);
                   end

               end
               
               self.UR.animate(URqMatrix(i,:));
               self.CO.animate(COqMatrix(i,:));

               COend = self.CO.fkine(self.CO.getpos).T;
               animateGripper(self.COgrip,COqGrip,COend);

               URend = self.UR.fkine(self.UR.getpos).T;
               animateGripper(self.URgrip,URqGrip,URend);

               if self.pendant.getColButStatus
                   self.CollCheck = self.EllipData.URmesh(URqMatrix(i,:)); %check for collisions with both bots
               end

               if self.CollCheck == true
                   self.pendant.actionCollisionDetection([self.UR.getpos; self.CO.getpos]);
                   self.CollCheck = false;
               end

               if self.pendant.getColButStatus
                   self.CollCheck = self.EllipData.CR5mesh(COqMatrix(i,:)); %check for collisions with both bots
               end

               if self.CollCheck == true
                   self.pendant.actionCollisionDetection([self.UR.getpos; self.CO.getpos]);
                   self.CollCheck = false;
               end

               if self.GlassLock == true
                   if self.GR == self.CO %if the current bot to hold the beer is the co, moves the beer with the co
                       grabvert = self.GR.fkine(COqMatrix(i,:)).T*self.CupMod;
                   end
                   if self.GR == self.UR %otherwise, move it with the UR
                       grabvert = self.GR.fkine(URqMatrix(i,:)).T*self.CupMod;
                   end
                   transVert = [self.v,ones(size(self.v,1),1)]*grabvert';
                   set(self.GP,'Vertices',transVert(:,1:3));
               end

               self.pendant.updateJointSliders;
               
           end
       end
      
       function ReleaseGlass(self,Gripper,qGrip) %function for opening the claw
            openClaw(Gripper) %play claw animation
            self.GlassLock = false; %tell ranimate that the glass should not move
            self.(qGrip) = Gripper.openClawPos; %update gipper jaw positions so they stay open when moving
            %qGrip takes text as an input so u can tell it which bots gripper position is in question
       end

       function GrabGlass(self,Robot,Gripper,qGrip) %function for closing the claw
            glassClaw(Gripper) %play claw animation
            self.GlassLock = true; %tell ranimate that the glass should move
            self.GP = self.glassPLY(self.glass); %tell ranimate which glass to move
            self.GR = Robot; %tell ranimate which robot should move the glass 
            self.(qGrip) = Gripper.glassClawPos; %update gipper jaw positions
       end

       function GrabTap(self,Gripper,qGrip) %function for grabing the tap with the claw
            tapClaw(Gripper) %play claw animation
            self.(qGrip) = Gripper.tapClawPos; %update gipper jaw positions
       end

       function BeerServer(self,GPick,GPlace) %Animation Loop pt1

           %to start
           q0 = [-1.6336,-1.1938,1.5080,-0.3142,1.5080,0]; %inital guess
           tr = transl(GPick+[0,0,0.2])*troty(pi/2)*trotx(-pi/2); %first position needed 
           newQ = self.UR.ikcon(tr,q0); 
           URqMatrix = jtraj(self.UR.getpos,newQ,50);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);
           %moves to first posion, uses jtraj for the 1st step so effective arm orientations are established

           %to cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T); %gets current xyz
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0;]'; %the anges the end effector should be at at each waypoint
           waypoints = [current; GPick;]'; %the 2 waypoints RMC will generate a path between
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,2); %RMC to genrate the qmatrix
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos); %ranimate along the q matrix

           self.GrabGlass(self.UR,self.URgrip,'URpos')

           %to tap
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0; -pi/2,pi/2,0;
               -pi,pi/2,0; (-270*pi/180),pi/2,0;]';
           waypoints = [current; (GPick+[0,0,0.2]);0,0.9,0.9;
               -0.05,0.5,0.8; GPlace;]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,3);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           self.ReleaseGlass(self.URgrip,'URpos')

           %grab tap
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-270*pi/180),pi/2,0;(-270*pi/180),pi/2,0;]';
           waypoints = [current; (GPlace+[0,0,0.25]);]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,2);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           self.GrabTap(self.URgrip,'URpos')

           %Tap cycle
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-270*pi/180),pi/2,0;(-270*pi/180),pi/2,0;]';
           waypoints = [current; (GPlace+[0,0.04,0.25]);(GPlace+[0,0.04,0.25]);
               (GPlace+[0,0,0.25])]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,3);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           self.ReleaseGlass(self.URgrip,'URpos')

           %To Cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-270*pi/180),pi/2,0;(-270*pi/180),pi/2,0;]';
           waypoints = [current; GPlace]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,2);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           self.GrabGlass(self.UR,self.URgrip,'URpos')

           %to end
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-270*pi/180),pi/2,0; (-270*pi/180),pi/2,0;
               (-310*pi/180),pi/2,0;]';
           waypoints = [current; 0.57,0.40,0.8;
               0.66,0.30,0.77]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,3);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos); 

           self.ReleaseGlass(self.URgrip,'URpos')

           %retract
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [(-310*pi/180),pi/2,0; (-310*pi/180),pi/2,0;
               (-2*pi),pi/2,0]';
           waypoints = [current; 0.57,0.40,0.8;
               0.75,0.66,1]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,2);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           q0 = [0,-pi/2,0,-pi/2,0,0];
           URqMatrix = jtraj(self.UR.getpos,q0,50);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);
       end

       function BeerRecaller(self,GPick,GPlace) %Animation Loop pt2
           %to start
           q0 = [-1.3823,-0.1257,-1.8396,-1.1310,-1.3823,0];
           tr = transl(GPlace+[0,0,0.2])*troty(pi/2)*trotx(pi/2);
           newQ = self.CO.ikcon(tr,q0);
           COqMatrix = jtraj(self.CO.getpos,newQ,50);
           self.Ranimate(self.CO,self.COgrip,COqMatrix,self.COpos);

           %to dirty cup
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi/2,pi/2,0; pi/2,pi/2,0;]';
           waypoints = [current; GPlace;]';
           COqMatrix = self.RMCgen(self.CO,waypoints,angles,2);
           self.Ranimate(self.CO,self.COgrip,COqMatrix,self.COpos);

           self.GrabGlass(self.CO,self.COgrip,'COpos')

           %to washer
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi/2,pi/2,0; pi/2,-pi/2,0; pi,-pi/2,0;
               3*pi/2,-pi/2,0; 3*pi/2,-pi/2,0;
               3*pi/2,-pi/2,0;3*pi/2,-pi/2,0;]';
           waypoints = [current; (GPlace+[0,0,0.2]);-0.2,0.65,0.9;
               GPick+[0,0,0.2]; (GPick+[0,0,0.04]);
               (GPick+[0,0,0.04]);GPick+[0,0,0.2];]';
           COqMatrix = self.RMCgen(self.CO,waypoints,angles,3);
           self.Ranimate(self.CO,self.COgrip,COqMatrix,self.COpos);

           %handover pt 1 (prepare)
           rendezvous = [-0.15,0.66,0.9]; %position were the two robots will meet to pass the glass

           q0 = [-0.1257,-0.2513,-1.4067,-1.5080,-1.7593,1.5080];
           tr = transl(rendezvous+[-0.18,0,0])*troty(-pi/2)*trotx(pi);
           newQ = self.CO.ikcon(tr,q0);
           COqMatrix = jtraj(self.CO.getpos,newQ,50);

           q0 = [0,-1.3195,1.8850,-0.5655,1.5708,1.5708];
           tr = transl(rendezvous+[0.18,0,0.03])*troty(-pi/2);
           newQ = self.UR.ikcon(tr,q0);
           URqMatrix = jtraj(self.UR.getpos,newQ,50);

           self.RanimateDual(URqMatrix,COqMatrix,self.URpos,self.COpos)

           %handover pt 2 (join)
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi,-pi/2,0; pi,-pi/2,0;]';
           waypoints = [current; (rendezvous+[-0.125,0,0]);]';
           COqMatrix = self.RMCgen(self.CO,waypoints,angles,1);

           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0;]';
           waypoints = [current; (rendezvous+[0.125,0,0.03]);]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,1);

           self.RanimateDual(URqMatrix,COqMatrix,self.URpos,self.COpos)

           self.GrabGlass(self.UR,self.URgrip,'URpos')
           self.ReleaseGlass(self.COgrip,'COpos')

           self.CupMod = troty(-pi/2)*transl(0.125,0,-0.04); %sets a temp new offset for the glass to be held
           %(because the UR must grab the glass at a difrent pos when being handed it from CO)
           self.GlassLock = true; %manually sets glass lock to true again (to negate self.releaseGlass setting it to false)

           %handover pt 3 (retract)
           current = tform2trvec(self.CO.fkine(self.CO.getpos).T);
           angles = [pi,-pi/2,0; pi,-pi/2,0;]';
           waypoints = [current; (rendezvous+[-0.18,0,0]);]';
           COqMatrix = self.RMCgen(self.CO,waypoints,angles,1);

           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0;]';
           waypoints = [current; (rendezvous+[0.18,0,0.03]);]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,1);

           self.RanimateDual(URqMatrix,COqMatrix,self.URpos,self.COpos)

       end

       function BeerReturner(self,GPick) %Animation Loop pt3
           %return cup
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [0,-pi/2,0; 0,-pi/2,0; -pi/2,pi/2,0;-pi/2,pi/2,0;]';
           waypoints = [current; current;(GPick+[0,0,0.2]);(GPick+[0,0,-0.03]);]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,3);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           self.ReleaseGlass(self.URgrip,'URpos')
           self.CupMod = troty(-pi/2)*transl(0.125,0,-0.07); %sets the glass offset back to its original point for the next run

           %cup retract
           current = tform2trvec(self.UR.fkine(self.UR.getpos).T);
           angles = [-pi/2,pi/2,0; -pi/2,pi/2,0; -pi/2,pi/2,0;]';
           waypoints = [current; (GPick+[0,0,0.2]);0.3,0.9,0.9;]';
           URqMatrix = self.RMCgen(self.UR,waypoints,angles,2);
           self.Ranimate(self.UR,self.URgrip,URqMatrix,self.URpos);

           %return to start
           q0 = [0,0,0,0,0,0]; %jtraj to get back to the start (easiest method, RMC not nessasary) 
           COqMatrix = jtraj(self.CO.getpos,q0,50);

           q0 = [0,-pi/2,0,-pi/2,0,0];
           URqMatrix = jtraj(self.UR.getpos,q0,50);

           self.RanimateDual(URqMatrix,COqMatrix,self.URpos,self.COpos)

       end

       function self = createEnvironment(self)  %includes everything needed to load the envirnoment

           O1 = PlaceObject('BarBase.ply',[0,0,0]); %load bar models
           O2 = PlaceObject('BarSafe.ply',[0,0,0]);
           PlaceObject('BeerTap.ply',[-0.05,0,0]); %all 4 taps
           PlaceObject('BeerTap.ply',[0.10,0,0]);
           PlaceObject('BeerTap.ply',[0.25,0,0]);
           PlaceObject('BeerTap.ply',[0.40,0,0]);

           self.glasspos = {[0.03,1.1,0.71],[0.16,1.1,0.71]...  %glass positions
               ,[0.29,1.1,0.71],[0.42,1.1,0.71]};

           self.glassPLY = [(PlaceObject('Glass.ply',self.glasspos{1,1}))...  %placing cups at starting positions
               ,(PlaceObject('Glass.ply',self.glasspos{1,2}))...
               ,(PlaceObject('Glass.ply',self.glasspos{1,3}))...
               ,(PlaceObject('Glass.ply',self.glasspos{1,4}))];

           self.cupSpot = {[-0.02,0.26,0.7],[0.13,0.26,0.7],[0.28,0.26,0.7]... %locations were the cup will be placed (under taps + cleaning bay + serving spot)
               ,[0.43,0.26,0.7],[0.73,0.20,0.7],[-0.68,0.20,0.7]...
               ,[-0.605,1.205,0.7]};

           [~,vec,~] = plyread('Glass.ply','tri');  %glass vector gata at 0,0,0
           self.v = vec;

           self.CupMod = troty(-pi/2)*transl(0.125,0,-0.07); %modifier added to the cup pos so it sits the correct distance from the gripper

           %which model vertices are to be considered in collision checks (exluding human model)
           self.trapvert = vertcat(get(O1,'Vertices'),get(O2,'Vertices'));

           %Creating the collision class object
           self.EllipData = Ellipsoid(self.UR,self.CO,self.trapvert);

       end

       function Pickpos = GlassReturn(self) %moves a glass into a random location in the return bay
           randomx = -0.5 + (-0.87 + 0.5)*rand; %gets random x and y locations within boundarys
           randomy = 0.15 + (0.30 - 0.15)*rand;
           Pickpos = [randomx,randomy,0.7];

           transVert = self.v+Pickpos;
           set(self.GP,'Vertices',transVert(:,1:3)); %moves glass to genrated coord
       end

    end

    methods (Static)
        function qMatrix = RMCgen(robot,waypoints,angles,t) %RMC path gen
            %%start/settings
            % t = 3;             % Total sim time (s)
            deltaT = 0.02;      % Control frequency (time between steps)
            steps = ceil(t/deltaT);   % No. of steps for simulation
            epsilon = 0.05;      % Threshold value for manipulability/Damped Least Squares (how close to sing before it modifyies path)
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
    end
end

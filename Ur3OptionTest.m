classdef Ur3OptionTest
    methods
        function self = Ur3OptionTest()
            clc
            clf
            %%r = UR3(transl([0,0.9,0.6])); %Load UR3 (with offset)
            r = UR3(); 
            r1 = r.model; %so you dont have to do .model every time
            hold on;
            % PlaceObject('BarBase.ply',[0,0,0]); %load bar models
            % PlaceObject('BarSafe.ply',[0,0,0]);
            % PlaceObject('BarBotBase.ply',[0,0,0]);
            % PlaceObject('BeerTap.ply',[0,0,0]);

            Glasspos = {[0.4,0,0],[0,0.4,0]};

            GlassPLY = [(PlaceObject('Glass.ply',Glasspos{1,1}))...  %placing bricks at starting location
                       ,(PlaceObject('Glass.ply',Glasspos{1,2}))];
          
            % axis([-2,2,-2,2,0,2]);
            view(3);
            axis equal;

            lock = true;
            glass = 2; %which glass is the target

            while true
                disp("Please select a point in space.");
                op = input("enter movment 1, 2 or 3. Select 4 to pick and 5 to place (enter 6 to exit program)...");
    
                switch op %remember to hit 4 to exit the program
                    case 1
                        disp("you selected 1");
                        q0 = [2.8903,-0.7540,1.7593,-1.0053,1.5080,0.0000]; %manually obtained guess
                        location = [0.4,0,0.05];
                        self.Robotmove(r1,q0,location,lock,GlassPLY,glass);
                    case 2 
                        disp("you selected 2");
                        q0 = [2.8903,-0.7540,1.7593,-1.0053,1.5080,0.0000]; %enter q0 guess
                        location = [-0.3,0,0.2]; %enter coords     
                        self.Robotmove(r1,q0,location,lock,GlassPLY,glass); %Animate function
                    case 3
                        disp("you selected 3");
                        q0 = [2.8903,-0.7540,1.7593,-1.0053,1.5080,0.0000];
                        location = [0,0.4,0.3];
                        self.Robotmove(r1,q0,location,lock,GlassPLY,glass);
                    case 4
                        disp("Picking up glass...");
                        lock = false;
                    case 5
                        disp("Placing glass...");
                        lock = true;
                    case 6
                        disp("Exiting");
                        break;
                    otherwise
                        disp("That was not an option!");
                end
            end
        end
    end

    methods (Static)
        function Robotmove(r1,q0,location,lock,GlassPLY,glass) %animates based on location inputs

            [f,v,data] = plyread('Glass.ply','tri'); %gets vector position data for glass ply at origin
   
            %angle between normal and target location
            r1base = r1.base().T;
            glassAngle = atan2((location(1,2)-r1base(2,4)),(location(1,1)-r1base(1,4)));

            tr = transl(location)*troty(pi/2)*trotx(-glassAngle); %makes x axis always face glass
            newQ = r1.ikcon(tr,q0);
            qMatrix = jtraj(r1.getpos,newQ,50);
            for i = 1:1:50 
                    r1.animate(qMatrix(i,:));
                    if lock == false
                        gripvert = r1.fkine(qMatrix(i,:)).T*troty(-pi/2); %gets current UR3 end effector position
                        transVert = [v,ones(size(v,1),1)]*gripvert'; %gets new values for the brick vertice positions
                        set(GlassPLY(1,glass),'Vertices',transVert(:,1:3)); %sets current brick vertices to new pos
                    end
                    pause(0.01);
            end
        end
    end
end
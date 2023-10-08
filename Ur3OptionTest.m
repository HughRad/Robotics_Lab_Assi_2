classdef Ur3OptionTest
    
    % properties
    %     Property1
    % end

    methods
        function self = Ur3OptionTest()
            clc
            clf

            r = UR3(transl([0,0.9,0.6])); %Load UR3 (with offset)
            r1 = r.model; %so you dont have to do .model every time
            hold on;
            PlaceObject('BarBase.ply',[0,0,0]); %load bar models
            PlaceObject('BarSafe.ply',[0,0,0]);
            PlaceObject('BarBotBase.ply',[0,0,0]);
            PlaceObject('BeerTap.ply',[0,0,0]);
            axis([-2,2,-2,2,0,2]);
            view(3);

            while true
                disp("Please select a point in space.");
                op = input("enter 1, 2 or 3 (enter 4 to exit program)...");
    
                switch op %remember to hit 4 to exit the program
                    case 1
                        disp("you selected 1");
                        q0 = [0,0,0,0,0,0];
                        location = [0.2,0.5,1];
                        self.Robotanimate(r1,q0,location);
                    case 2 
                        disp("you selected 2");
                        q0 = [0,0,0,0,0,0]; %enter q0 guess
                        location = [-0.2,1.4,1]; %enter coords
                        self.Robotanimate(r1,q0,location); %Animate function
                    case 3
                        disp("you selected 3");
                        q0 = [0,0,0,0,0,0];
                        location = [-0.5,1,1];
                        self.Robotanimate(r1,q0,location);
                    case 4
                        disp("Exiting");
                        break;
                    otherwise
                        disp("That was not an option!");
                end
            end
        end
    end

    methods (Static)
        function Robotanimate(r1,q0,location) %animates based on location inputs
            tr = transl(location);
            newQ = r1.ikcon(tr,q0);
            qMatrix = jtraj(r1.getpos,newQ,50);
            for i = 1:1:50 
                    r1.animate(qMatrix(i,:));
                    pause(0.01);
            end
        end
    end
end
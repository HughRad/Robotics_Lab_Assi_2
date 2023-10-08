classdef Ur3OptionTest
    
    % properties
    %     Property1
    % end

    methods
        function self = Ur3OptionTest()
            clc
            clf

            r = UR3(transl([0,0.9,0.6]));
            r1 = r.model;
            hold on;
            PlaceObject('BarBase.ply',[0,0,0]);
            PlaceObject('BarSafe.ply',[0,0,0]);
            PlaceObject('BarBotBase.ply',[0,0,0]);
            PlaceObject('BeerTap.ply',[0,0,0]);
            axis([-2,2,-2,2,0,2]);


            while true
                disp("Please select a point in space.");
                op = input("enter 1, 2 or 3 (enter 4 to exit program)...");
    
                switch op
                    case 1
                        disp("you selected 1");
                        q0 = r1.getpos;
                        tr = transl([0.2,0.5,1]);
                        newQ = r1.ikcon(tr,q0);
                        self.Robotanimate(r1,q0,newQ);
                    case 2 
                        disp("you selected 2");
                        q0 = r1.getpos;
                        tr = transl([-0.2,1.4,1]);
                        newQ = r1.ikcon(tr,q0);
                        self.Robotanimate(r1,q0,newQ);
                    case 3
                        disp("you selected 3");
                        q0 = r1.getpos;
                        tr = transl([-0.5,1,1]);
                        newQ = r1.ikcon(tr,q0);
                        % r.model.animate(newQ);
                        self.Robotanimate(r1,q0,newQ);
                    case 4
                        disp("Exiting");
                        break;
                    otherwise
                        disp("That was not an option!");
                end
            end
            % self.Pos1() 
        end
    end

    methods (Static)
        function Robotanimate(r1,q0,newQ)
            qMatrix = jtraj(q0,newQ,50);
            for i = 1:1:50 
                    r1.animate(qMatrix(i,:));
                    pause(0.01);
            end
        end
    end
end
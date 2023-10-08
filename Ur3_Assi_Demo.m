classdef Ur3_Assi_Demo
    %Robot Assignment - Hugh Radvan - 13549340
    %UR3 Linear Operations demonstration class

    properties (Constant)      %brick end positions
        bricklocfi = {[0.4,0,0],[0.4,-0.14,0],[0.4,0.14,0]... 
                   ,[0.4,0,0.035],[0.4,-0.14,0.035],[0.4,0.14,0.035]...
                   ,[0.4,0,0.07],[0.4,-0.14,0.07],[0.4,0.14,0.07]};
            
                                %brick start positions
        brickloc = {[-0.77, -0.4, 0],[-0.6, -0.4, 0],[-0.45, -0.4, 0]... 
                   ,[-0.3, -0.4, 0],[-0.77, 0.4, 0],[-0.6, 0.4, 0]...
                   ,[-0.45, 0.4, 0],[-0.3, 0.4, 0],[-0.15, 0.4, 0]};
    end

    methods
        function self = Ur3_Assi_Demo()
            % self.RobotTest() %used to test UR3 robot operations 
            % input('Press enter to begin workspace volume calculations')
            % self.WorkspaceVolume() %method to calculate UR3 reah area and volume  
            % input('Press enter to begin demo')
            self.RobotDemo() %method to play brick stacking demo
            clf
            clc
        end
    end

    methods (Static)
        function RobotDemo()
            clf
            clc

            %Gripper + UR3 joint setup
            qf1 = [deg2rad(30), deg2rad(60)]; %inital gripper joint angles (finger 1 + 2)
            qf2 = [deg2rad(150), deg2rad(-60)];
            qf11 = [deg2rad(60), deg2rad(30)];
            qf22 = [deg2rad(120), deg2rad(-30)];
            q0 = [-0.0653,1.3823,0.6283,1.6022,-0.6283,-1.5080,0.0000]; %inital UR3 joint guess (for brick pick)
            q00 = [-0.0100,2.2171,-0.6912,-1.3648,0.5027,1.5080,1.2566]; %initial UR3 joint guess (for brick place)
            q000 = [-0.02,0,0,0,0,0,0]; %inital and final UR3 joints 

            qMxf1c = jtraj(qf1,qf11,50); %qMatrix for the grippers (open/close)
            qMxf2c = jtraj(qf2,qf22,50);
            qMxf1o = jtraj(qf11,qf1,50);
            qMxf2o = jtraj(qf22,qf2,50);
            
                %Robot ploting 
            r = LinearUR3(); %plot UR3
            hold on
            fi = Gripper(); %Plot Gripper Fingers
            fi.finger1.delay=0; %minimising animation delay
            fi.finger2.delay=0;
            r.model.delay=0;
            grip = PlaceObject('Gripper.ply',[0,0,0]); %load gripper model
            [f1,v1,data1] = plyread('Gripper.ply','tri'); %gets gripper vertex info at 0,0,0
            
            fi.finger1.base = (r.model.fkine(q000).T)*trotx(pi/2)*transl(0,0.1,0.015); %Setting Finger Base to end of UR3
            fi.finger2.base = (r.model.fkine(q000).T)*trotx(pi/2)*transl(0,0.1,0.015); %fkine gets xyz of UR3 end effector
            fi.finger1.animate(qf1); %moving fingers to end effector
            fi.finger2.animate(qf2);
            
            %section sets gripper vertices to UR3 end effector
            gripvertfin = r.model.fkine(r.model.getpos).T*trotz(pi/2)*transl(0,0,-0.05); 
            transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
            set(grip,'Vertices',transVertfin(:,1:3)); 
            pause(0.05);
            
            %Initializing Environment and props
            
                %Environment setup
            hold on
            
            PlaceObject('Pillar.ply',[1.5,2,0]); %Generating Pillars
            PlaceObject('Pillar.ply',[1.5,-2,0]);
            PlaceObject('Pillar.ply',[-2.5,2,0]);
            PlaceObject('Pillar.ply',[-2.5,-2,0]);
            PlaceObject('Scene.ply',[-0.5,0,0]); %Generating the remaining scene
            
            axis equal;
            
                %Brick Loading
            
            [f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri'); %gets vector position data for brick at origin
            
            bricklocfi = Ur3_Assi_Demo.bricklocfi;
            
            brickloc = Ur3_Assi_Demo.brickloc;
            
            brickpla = [(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,1}))...  %placing bricks at starting location
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,2}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,3}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,4}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,5}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,6}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,7}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,8}))...
                       ,(PlaceObject('HalfSizedRedGreenBrick.ply',brickloc{1,9}))];
            
            hold on;
            axis([-2.8,1.8,-2.3,2.3,0,1.3]); %set axis
            %axis([-1,0.5,-0.7,0.7,0,0.8]); %close up
            hold off;

            
            %Demo Animation
                %Brick loop
            
            q1 = q000; %sets q1 to UR3 starting location joint angles

            diary UR3_BrickDemo_Output.txt; %logs command window inputs and outputs to txt file
            disp('Commencing brick placing demo. Output log start:')
            
            for h = 1:1:9 %iterates loop for all 9 bricks
            
                %moving UR3 to brick starting location --------------------------------
            
                tr = transl(brickloc{1,h})*troty(pi)*transl(0,0,-0.22); %gets transform for brick starting location
                %Adjusts transform by 180 on the y axis to brick is grabbed from the top
                %Adjusts transform with Zoffset to compensate for the
                %gripper size
            
                newQ = r.model.ikcon(tr,q0); %gets UR3 joint angles to reach brick starting location 
                %ikcon used to take joint limits into account, q0=preset inital guess
            
                qMatrix = jtraj(q1,newQ,50); %matrix of joint values generated to get from 
                %current 'q1' to desired 'newQ'. qunitic polynomial method used to
                %reduce jerk (we do not care about the robots position between a and b)

                disp(['UR3 moving to brick ',num2str(h),' out of 9'])
                
                for i = 1:1:50 %iterating the 50 qmatrix joint positions
                    fi.finger1.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %updating Finger Base to end of UR3
                    fi.finger2.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %at the new q value
                    r.model.animate(qMatrix(i,:)); %animating UR3 to new joint positions from qmatrix
                    fi.finger1.animate(qf1); %animating gripper fingers to new base
                    fi.finger2.animate(qf2);

                    %animating gripper fingers closing during movement to
                    %brick
                    fi.finger1.animate(qMxf1c(i,:));
                    fi.finger2.animate(qMxf2c(i,:));

                    %section sets gripper vertices to UR3 end effector
                    gripvertfin = r.model.fkine(qMatrix(i,:)).T*trotz(pi/2)*transl(0,0,-0.05); 
                    transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                    set(grip,'Vertices',transVertfin(:,1:3)); 

                    pause(0.05);
                end
                
                disp(['Brick ',num2str(h),' has been grabbed at the following transform:'])
                Current_UR3_Pick_Transform = r.model.fkine(r.model.getpos).T*transl(0,0,0.22) %get current UR3 transform (ajusted for offset)

                %moving UR3 and brick to stacking location ----------------------------
                disp(['UR3 moving brick ',num2str(h),' to wall location'])
            
                q1 = newQ; %sets q1 to current joint angles from last step
                tr = transl([bricklocfi{1,h}])*troty(pi)*transl(0,0,-0.22); %gets transfrom to brick stack location
                newQ = r.model.ikcon(tr,q00); %gets ur3 joint angles to reach new transform
                qMatrix = jtraj(q1,newQ,50); %creates a joint angle matrix from q1 to newQ 
                
                
                for i = 1:1:50 %iterating the 50 qmatrix joint positions
                    fi.finger1.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %Updates finger and UR3 locations
                    fi.finger2.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %same as last step
                    r.model.animate(qMatrix(i,:));
                    fi.finger1.animate(qf11);
                    fi.finger2.animate(qf22);

                    %section sets gripper vertices to UR3 end effector 
                    gripvertfin = r.model.fkine(qMatrix(i,:)).T*trotz(pi/2)*transl(0,0,-0.05); 
                    transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                    set(grip,'Vertices',transVertfin(:,1:3)); 
                    
                    %moving the brick
                    gripvert = r.model.fkine(qMatrix(i,:)).T*transl(0,0,0.18); %gets current UR3 end effector position
                    transVert = [v,ones(size(v,1),1)]*gripvert'; %gets new values for the brick vertice positions
                    %using the the vector data for a brick at the origin
                    %vector matrix size modified to be compatible with the transform matrix 
                    set(brickpla(1,h),'Vertices',transVert(:,1:3)); %sets current brick vertices 
                                                                    %to the transformed vertices
                    q1 = r.model.getpos; %sets q1 to current UR3 joint angles
                    pause(0.05);
                end
                
                disp(['Brick ',num2str(h),' has been deposited on the wall at the following transform:'])
                Current_UR3_Place_Transform = r.model.fkine(r.model.getpos).T*transl(0,0,0.22)

                for i = 1:1:50 %finger opening/releasing brick when stopped
                    fi.finger1.animate(qMxf1o(i,:));
                    fi.finger2.animate(qMxf2o(i,:));
                    pause(0.02);
                end
                
            end

            disp('Brick stacking complete, moving UR3 to starting position. Output log end...')
            diary off;

            %UR3 return animation
            
                qMatrix = jtraj(q1,q000,50); %q matrix to get from current position to start position
            
                for i = 1:1:50
                    fi.finger1.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %Updates finger and UR3 locations 
                    fi.finger2.base = (r.model.fkine(qMatrix(i,:)).T)*trotx(pi/2)*transl(0,0.1,0.015); %same as last step
                    r.model.animate(qMatrix(i,:));
                    fi.finger1.animate(qf1);
                    fi.finger2.animate(qf2);

                    %section sets gripper vertices to UR3 end effector
                    gripvertfin = r.model.fkine(qMatrix(i,:)).T*trotz(pi/2)*transl(0,0,-0.05); 
                    transVertfin = [v1,ones(size(v1,1),1)]*gripvertfin'; 
                    set(grip,'Vertices',transVertfin(:,1:3)); 

                    pause(0.05);
                end
            input('Press enter to end demo')
        end

        function WorkspaceVolume()
            clf
            clc
            xt = 0;
            yt = 0;
            zt = 0;
            transform = [xt,yt,zt]; %custom UR3 base transform
            hold on
            PlaceObject('Pillar.ply',(transform+[1.5,2,0])); %Generating Pillars
            PlaceObject('Pillar.ply',(transform+[1.5,-2,0]));
            PlaceObject('Pillar.ply',(transform+[-2.5,2,0]));
            PlaceObject('Pillar.ply',(transform+[-2.5,-2,0]));
            PlaceObject('Scene.ply',(transform+[-0.5,0,0])); %Generating the remaining scene
            r = LinearUR3(transl(transform)); %plot UR3

            %point cloud plotting (based on lab 3)
            sRad = deg2rad(45); %define the increments at which we will take spatial measurements
            jl = r.model.qlim; %recalling the UR3 joint limits
            pCloudeSize = prod(floor((jl(1:6,2)-jl(1:6,1))/sRad + 1)); %pre-calculating cloud size
            pointCloud = zeros(pCloudeSize,3); %creating an array of zero doubles (size x 3)
            counter = 1; %for keeping track of progress through cloud

            %iterating each joint between limits by sRad increments
            for q1 = jl(1,1):0.79:jl(1,2)
                for q2 = jl(2,1):sRad:jl(2,2)
                    for q3 = jl(3,1):sRad:jl(3,2)
                        for q4 = jl(4,1):sRad:jl(4,2)
                            for q5 = jl(5,1):sRad:jl(5,2)
                                for q6 = jl(6,1):sRad:jl(6,2)
                                    q7 = 0; %this joint makes no difference to UR3 position
                                    q = [q1,q2,q3,q4,q5,q6,q7]; %sets UR3 joint to current for loop position
                                    tr = r.model.fkineUTS(q); %gets transform matrix at current q                       
                                    pointCloud(counter,:) = tr(1:3,4)'; %sets current cloud position to be the xyz of the tr matrix 
                                    counter = counter + 1; %updates counter
                                    if mod(counter/pCloudeSize * 100,1) == 0 %specifies to give updates on whole numbers
                                        disp(['Calculations ',num2str(counter/pCloudeSize * 100),'% complete']);
                                    %gives current status of point cloud pose calculations
                                    end
                                end
                            end
                        end
                    end
                end
            end

            OOBz = pointCloud(:,3) < (0+zt); %selecting all values below z=0
            pointCloud(OOBz, :) = []; %setting them to = 0 as they are below the floor

            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.'); %plotting the point cloud
            [k,av]=convhull(pointCloud); %finding the convex hull of the point cloud (max volume)
            disp(['Aprox reach volume of UR3 (m^3) = ',num2str(av)]); %displaying point cloud volume
            input('Press enter to to see workspace area calculations')

            clf
            hold on
            PlaceObject('Pillar.ply',(transform+[1.5,2,0])); %Generating Pillars
            PlaceObject('Pillar.ply',(transform+[1.5,-2,0]));
            PlaceObject('Pillar.ply',(transform+[-2.5,2,0]));
            PlaceObject('Pillar.ply',(transform+[-2.5,-2,0]));
            PlaceObject('Scene.ply',(transform+[-0.5,0,0])); %Generating the remaining scene
            r = LinearUR3(transl(transform)); %plot UR3

            pointCloudxy = pointCloud(:,1:2);
            pointCloud(:, 3) = (0.02 +zt); %reduces point cloud to the x-y plane (at a small z value to be visable over the workspace)
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'g.'); %plotting the xy point cloud
            [k,av]=convhull(pointCloudxy); %finding the convex hull of the point cloud at z=0 (max area)
            disp(['Aprox reach area of UR3 (m^2) = ',num2str(av)]); %displaying z=0 point cloud area
 
            input('Press enter to complete workspace calculations')
        end
    
        function RobotTest()
            clc
            clf
            input('Press enter to begin UR3 Test')
            r = LinearUR3(); %plot UR3
            hold on;
            q0 = [0,0,0,0,0,0,0];
            r.model.teach(q0);
            axis([-2,1,-1,1,0,1]);

            input('Press enter to move to given position')
            inPose = [-1, 0.3, 0.3] %given pose coordinates
            tr = transl(inPose)*troty(pi); %finds transform to given coordinates
            newQ = r.model.ikcon(tr,q0) %finds joint angles to reach pose (inital guess q0)

            oldPos = r.model.getpos; %gets current q values
            qMatrix = jtraj(oldPos,newQ,50); %jtraj from start to end q
            
            for i = 1:1:50 %iterating the 50 qmatrix joint positions
                r.model.animate(qMatrix(i,:)); %animating UR3 to new joint position
                pause(0.05);
            end
            
            outTr = r.model.fkine(r.model.getpos).T; %final transfomation matrix
            outPose = outTr(1:3,4)' %get xyz of final position
            Difference = round((abs(outPose) - abs(inPose)),4) %Difference rounded to 4dp

            input('Press enter to move to finish test')

        end
    end
end
classdef Place2F85
    properties
        robot;
        fingers;
        toolB;
        clawbase;
        fingerJointPos;
        openClawPos;
        tapClawPos;
        glassClawPos;
        fingerBase;  % Precomputed fingerBase transformation (
        fingerBaseTransforms;  % Precomputef base transformations for each finger
        gripperBaseVertices;  % Precomputed gripperBase vertices
    end

    methods
        function place = Place2F85(robotModel)
            clc;
            place.robot = robotModel;
            place.fingers = {R2F85(), R2F85()};

            place.openClawPos = [0, 0];     % Define open position
            place.tapClawPos = [deg2rad(33), deg2rad(-25.5)]; % Define tap pos 
            place.glassClawPos = [deg2rad(12),deg2rad(6.2)]; %Define glass pos
            place.fingerJointPos = place.fingers{1}.model.getpos;

            for i = 1:numel(place.fingers)
                place.fingers{i}.model.delay = 0;
            end

            % Calculate the fingerBase transformation
            toolB = place.robot.fkine(place.robot.getpos).T;
            Offset = transl(0.05642, 0, 0);
            place.fingerBase = toolB * Offset;

            % Precompute base transformations for each finger
            place.fingerBaseTransforms = {
                place.calculateFingerBase(1, place.fingerBase),...
                place.calculateFingerBase(2, place.fingerBase),...
            };

            % Load the 'GripperBase.ply' file and vertices
            [place.clawbase, place.gripperBaseVertices] = place.loadGripperBase();
            
            
            % sets base of each finger from pre computed transforms
            for i = 1:numel(place.fingers)
                place.fingers{i}.model.base = place.fingerBaseTransforms{i};
            end
        end

        function animateGripper(place,qPos,newToolB)        
            % Calculate the base transformation for the Finger objects based on the newToolB
            for i = 1:numel(place.fingers)
                place.fingers{i}.model.base = place.calculateFingerBase(i, newToolB);
                place.fingers{i}.model.animate(qPos);
            end
            % Update the gripper base position
            place.updateGripperBaseVertices(newToolB);
        end

        % determines new base transformation for each finger based on the new toolbase 
        %(Robots end effector transform)
        function base = calculateFingerBase(place, fingerIndex, newToolB)
             if nargin < 3
                toolB = place.robot.model.fkine(place.robot.model.getpos).T;
             else
                toolB = newToolB;
             end
            Offset = transl(0, 0, 0.05642);
            FingerBase = newToolB*Offset;
            baseTransforms = {
                FingerBase * transl(0, 0.01270,0)*trotx(deg2rad(180))*troty(deg2rad(90)),...
                FingerBase * transl(0, -0.01270, 0)*troty(deg2rad(-90))...
            };
            base = baseTransforms{fingerIndex};
        end

        %loads in base of gripper and stores vertices
        function [clawbase, vertices] = loadGripperBase(place)
            clawbase = PlaceObject('R2F85Base.ply', [0, 0, 0]);
            [~, vertices, ~] = plyread('R2F85Base.ply', 'tri');
        end

        %Updates base of gripper from the placed position to the inital tool base 
        %transform base vertices by the robot end effector transform
        function updateGripperBaseVertices(place, newToolB)
            clawBase = newToolB * trotz(deg2rad(-180))*trotx(deg2rad(90))*troty(deg2rad(90));
            baseVert = [place.gripperBaseVertices, ones(size(place.gripperBaseVertices, 1), 1)] * clawBase';
            set(place.clawbase, 'Vertices', baseVert(:, 1:3));
        end

        % animates gripper from current joint position to open joint position
        function openClaw(place)
            steps = 50;
            place.fingerJointPos = place.fingers{1}.model.getpos;
            cM = jtraj(place.fingerJointPos,place.openClawPos,steps);
            for j = 1:steps
                for i = 1:numel(place.fingers)
                 place.fingers{i}.model.animate(cM(j,:));
                end
                pause(0.02)
            end
            drawnow()  
        end

        % animates gripper from current joint position to tap joint position
        function tapClaw(place) 
            steps = 50;
            place.fingerJointPos = place.fingers{1}.model.getpos;
            cM = jtraj(place.fingerJointPos,place.tapClawPos,steps);
            for j = 1:steps
                for i = 1:numel(place.fingers)
                 place.fingers{i}.model.animate(cM(j,:));
                end
                pause(0.02)
            end
            drawnow()
        end

        % animates gripper from current joint position to glass joint position
        function glassClaw(place)
            steps = 50;
            place.fingerJointPos = place.fingers{1}.model.getpos;
            cM = jtraj(place.fingerJointPos,place.glassClawPos,steps);
            for j = 1:steps
                for i = 1:numel(place.fingers)
                 place.fingers{i}.model.animate(cM(j,:));
                end
                pause(0.02)
            end
            drawnow()
        end

    end
end

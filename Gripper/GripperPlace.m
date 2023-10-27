classdef GripperPlace
    properties
        robot;
        fingers;
        toolB;
        clawbase;
        fingerJointPos;
        openClawPos;
        tapClawPos;
        glassClawPos;
        fingerBase;  % Precompute fingerBase transformation
        fingerBaseTransforms;  % Precompute base transformations for each finger
        gripperBaseVertices;  % Precompute gripperBase vertices
    end

    methods
        function place = GripperPlace(robotModel)
            clc;
            place.robot = robotModel;
            place.fingers = {Finger(), Finger(), Finger()};

            place.openClawPos = [0, 0, 0];     % Define open position
            place.tapClawPos = [0.1, 0.1, 0.1]; % Define tap pos 
            place.glassClawPos = [0.2, 0.1, 0.1]; %Define glass pos
            place.fingerJointPos = place.fingers{1}.model.getpos;

            for i = 1:numel(place.fingers)
                place.fingers{i}.model.delay = 0;
            end

            % Calculate the fingerBase transformation
            toolB = place.robot.fkine(place.robot.getpos).T;
            Offset = transl(0, 0, 0.09);
            place.fingerBase = toolB * Offset;

            % Precompute base transformations for each finger
            place.fingerBaseTransforms = {
                place.calculateFingerBase(1, place.fingerBase),...
                place.calculateFingerBase(2, place.fingerBase),...
                place.calculateFingerBase(3, place.fingerBase)
            };

            % Load the 'GripperBase.ply' file and vertices
            [place.clawbase, place.gripperBaseVertices] = place.loadGripperBase();

            % Teach initial joint positions for each Finger
            for i = 1:numel(place.fingers)
                place.fingers{i}.model.base = place.fingerBaseTransforms{i};
                place.fingers{i}.model.teach(place.openClawPos);
            end
        end

        function animateGripper(place, fingerJointPos, newToolB)        
            % Calculate the base transformation for the Finger objects based on the newToolB
            for i = 1:numel(place.fingers)
                place.fingers{i}.model.base = place.calculateFingerBase(i, newToolB);
                place.fingers{i}.model.animate(fingerJointPos);
            end
            % Update the gripper base position
            place.updateGripperBaseVertices(newToolB);
        end

        function base = calculateFingerBase(place, fingerIndex, newToolB)
             if nargin < 3
                toolB = place.robot.model.fkine(place.robot.model.getpos).T;
             else
                toolB = newToolB;
             end
            Offset = transl(0, 0, 0.09);
            FingerBase = newToolB*Offset;
            baseTransforms = {
                FingerBase * transl(0.0, -0.04, 0) * trotx(deg2rad(90)) * troty(deg2rad(-90)),...
                FingerBase * transl(0.035, 0.04, 0) * trotx(deg2rad(90)) * troty(deg2rad(90)),...
                FingerBase * transl(-0.035, 0.04, 0) * trotx(deg2rad(90)) * troty(deg2rad(90))
            };
            base = baseTransforms{fingerIndex};
        end

        function [clawbase, vertices] = loadGripperBase(place)
            clawbase = PlaceObject('GripperBase.ply', [0, 0, 0]);
            [~, vertices, ~] = plyread('GripperBase.ply', 'tri');
        end

        function updateGripperBaseVertices(place, newToolB)
            clawBase = newToolB * trotz(deg2rad(-90));
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
classdef realUR3Controller < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        jointStateSubscriber;

        jointStates;
    end

    methods
        function self = realUR3Controller
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            self.init
        end

        function init(self)
            %% Initialise connection to the ROS computer
            % To initialise a connection to the ROS computer from Matlab,
            % call rosinit to the correct IP address, and then start to
            % subscribe to the joint states

            self.jointStates = {[0,-90,0,-90,0,0], ...
                [0,-60,78,-14,90,0], ...
                [0,-40,90,-45,90,0], ...
                [0,-60,78,-14,90,0], ...
                [-220, -20, 37, -16, 51, 0], ...
                [-228, -41, 30, 12, 43, 0], ...
                [-244, -65, 63, 2, 27, 0], ...
                [-228, -41, 30, 12, 43, 0], ...
                [-220, -20, 37, -16, 51, 0], ...
                [-157, -3, 10, -5, 79, 0], ...
                [-157, -38, 84, -46, 81, 0], ...
                [0,-90,0,-90,0,0]};

            rosshutdown;
            rosinit('192.168.27.1'); % If unsure, please ask a tutor
            self.jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');


        end

        function runDemo(self)

            for i = 2:length(self.jointStates)

                self.goTo(self.jointStates{i});
            end

        end

        function goTo(self,q)

            %% Get current state from the real robot
            % To get the current joint state from the real robot

            self.jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
            pause(2); % Pause to give time for a message to appear
            currentJointState_321456 = (self.jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
            currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
            % If this fails then try to see what is in the latest message. If it is empty then you are not connected properly.

            self.jointStateSubscriber.LatestMessage
            
            
            %% Create information to send

            % Before sending commands, we create a variable with the joint names so that the joint commands are associated with a particular joint.

            jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            goal.Trajectory.JointNames = jointNames;
            goal.Trajectory.Header.Seq = 1;
            goal.Trajectory.Header.Stamp = rostime('Now','system');
            goal.GoalTimeTolerance = rosduration(0.05);
            bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
            durationSeconds = 5; % This is how many seconds the movement will take

            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            startJointSend.Positions = currentJointState_123456;
            startJointSend.TimeFromStart = rosduration(0);

            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            nextJointState_123456 = q;
            endJointSend.Positions = nextJointState_123456;
            endJointSend.TimeFromStart = rosduration(durationSeconds);

            goal.Trajectory.Points = [startJointSend; endJointSend];
            
            

            goal.Trajectory.Header.Stamp = self.jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
            sendGoal(client,goal);
            input('Press enter to continue');

        end

    end
end
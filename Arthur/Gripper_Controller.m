classdef Gripper_Controller < handle
    %GRIPPER_CONTROLLER Used for opening/closing the gripper

    properties (Access = private)
        q =zeros(1,3);     % both fingers will have same joint angles
        STEPS = 10;                 % number of steps to move in the animation

        Q_OPEN; % open position joint angles
        Q_CLOSED; % closed position joint angles

        status = 1; % progress status for incrementing movement of end effector

    end

    properties (Access = public)
        gripperStatus = 0;          % open = 0, closed = 1
        rightGripper; % selfect for the right gripper
        leftGripper; % selfect for the left gripper        
        
    end

    methods
        %% Gripper constructor
        function self = Gripper_Controller()
            %GRIPPER_CONTROLLER
            %   initialise the gripper
            
            Gripper;

            self.rightGripper=gripperRight;
            self.leftGripper=gripperLeft;

            self.Q_OPEN=qOpen;
            self.Q_CLOSED=qClosed;
            
            
            %plot the gripper
            hold on
            self.rightGripper.plot(qOpen);
            self.leftGripper.plot(qOpen);

            %update joint angles
            self.q = self.rightGripper.getpos;
        end

        %% Open gripper
        function [complete] = open(self)

            complete = 0;

            if self.gripperStatus == 0
                complete = 1;
                return
            end


            hold on
            self.rightGripper.plot(self.rightGripper.getpos);
            self.leftGripper.plot(self.leftGripper.getpos);

            self.q=self.rightGripper.getpos;

            qMatrix=jtraj(self.q,self.Q_OPEN,self.STEPS);

            % animate the step
            self.leftGripper.animate(qMatrix(self.status,:));
            self.rightGripper.animate(qMatrix(self.status,:));

            % handle the case where the movement is finished
            if self.status == self.STEPS

                % set the task to complete
                complete = 1;

                % reset the step counter
                self.status = 1;

                % update internal variables
                self.q = self.Q_OPEN;
                self.gripperStatus=0;
            end

            self.status=self.status+1;

        end

        %% Close gripper
        function [complete] = close(self)

            complete = 0;

            if self.gripperStatus == 1
                complete = 1;
                return
            end

            hold on
            self.rightGripper.plot(self.rightGripper.getpos);
            self.leftGripper.plot(self.leftGripper.getpos);

            self.q=self.rightGripper.getpos;

            qMatrix=jtraj(self.q,self.Q_CLOSED,self.STEPS);

            % animate the step
            self.leftGripper.animate(qMatrix(self.status,:));
            self.rightGripper.animate(qMatrix(self.status,:));

            % handle the case where the movement is finished
            if self.status == self.STEPS

                % set the task to complete
                complete = 1;

                % reset the step counter
                self.status = 1;

                % update internal variables
                self.q = self.Q_CLOSED;
                self.gripperStatus=1;
            end

            self.status=self.status+1;

        end

        %% Getter for the joint angles
        function [q] = getJointAngles(self)

            q=self.updateJointAngles;

        end

        function [q]=updateJointAngles(self)
            
            try 
                self.q=self.rightGripper.getpos;
                q=self.q;
            catch 
                disp('unable to find joint angles - plot is likely closed');
            end

        end
        %% Update base location
        function updateBaseLocation(self,T)

            self.rightGripper.base=T*transl(0.01,0,0)*trotz(deg2rad(180));
            self.leftGripper.base=T*transl(-0.01,0,0);
            self.rightGripper.animate(self.updateJointAngles);
            self.leftGripper.animate(self.updateJointAngles);

        end


    end

end


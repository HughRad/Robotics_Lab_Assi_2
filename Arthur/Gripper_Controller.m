classdef Gripper_Controller < handle
    %GRIPPER_CONTROLLER Used for opening/closing the gripper

    properties (Access = private)
        q =zeros(1,4);     % both fingers will have same joint angles
        STEPS = 10;                 % number of steps to move in the animation

        Q_OPEN; % open position joint angles
        Q_CLOSED; % closed position joint angles

        status = 1; % progress status for incrementing movement of end effector

    end

    properties (Access = public)
        gripperStatus = 0;          % open = 0, closed = 1
        A; % object for the thumb gripper
        B; % object for the index gripper
        C; % object for the pinky gripper        
        
    end

    methods
        %% Gripper constructor
        function self = Gripper_Controller()
            %GRIPPER_CONTROLLER
            %   initialise the gripper
            
            Gripper;

            self.A = A;
            self.B = B;
            self.C = C;

            self.Q_OPEN=qOpen;
            self.Q_CLOSED=qClosed;
            
            
            %plot the gripper
            hold on
            self.A.plot(qOpen,'nowrist','nobase');
            self.B.plot(qOpen,'nowrist','nobase');
            self.C.plot(qOpen,'nowrist','nobase');
            axis equal

            %update joint angles
            self.q = self.A.getpos;
        end

        %% Open gripper
        function [complete] = open(self)

            complete = 0;

            if self.gripperStatus == 0
                complete = 1;
                return
            end


            hold on
            self.A.plot(self.A.getpos);
            self.B.plot(self.B.getpos);
            self.C.plot(self.C.getpos);

            self.q=self.A.getpos;

            qMatrix=jtraj(self.q,self.Q_OPEN,self.STEPS);

            % animate the step
            self.A.animate(qMatrix(self.status,:));
            self.B.animate(qMatrix(self.status,:));
            self.C.animate(qMatrix(self.status,:));

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
            self.A.plot(self.A.getpos);
            self.B.plot(self.B.getpos);
            self.C.plot(self.C.getpos);

            self.q=self.A.getpos;

            qMatrix=jtraj(self.q,self.Q_CLOSED,self.STEPS);

            % animate the step
            self.B.animate(qMatrix(self.status,:));
            self.A.animate(qMatrix(self.status,:));
            self.C.animate(qMatrix(self.status,:));

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
                self.q=self.A.getpos;
                q=self.q;
            catch 
                disp('unable to find joint angles - plot is likely closed');
            end

        end
        %% Update base location
        function updateBaseLocation(self,T)

            self.A.base = transl(0.0,  -0.04,   0) * trotz(deg2rad(90));
            self.B.base = transl(0.055,  0.04, 0) * trotz(deg2rad(-90));
            self.C.base = transl(-0.055, 0.04, 0) * trotz(deg2rad(-90));

            self.A.animate(self.updateJointAngles);
            self.B.animate(self.updateJointAngles);
            self.C.animate(self.updateJointAngles);

        end


    end

end


classdef tester
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        count;
        app;
    end

    methods
        function self = tester(app)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            self.app = app;
            self.count = 1;
        end

        function printCount(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            while true
                self.count
                self.count = self.count + 1;
                pause(1);
                if ~isempty(self.app)
                    checkPauseStopStatus(self.app);
                end
            end
        end
    end
end
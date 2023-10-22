classdef DobotCR5 < RobotBaseClass
    properties(Access = public)   
        plyFileNameStem = 'DobotCR5';
    end
    methods
%% Constructor
        function self = DobotCR5(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.1348,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0.1288,'a', -0.274,'alpha',0 ,'qlim', deg2rad([-360 360]), 'offset',-pi/2);
            link(3) = Link('d',-0.1165,'a',0.230,'alpha',0,'qlim', deg2rad([-155 155]), 'offset', pi);
            link(4) = Link('d',0.116,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', pi/2);
            link(5) = Link('d',0.116,'a', 0 ,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.105,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end

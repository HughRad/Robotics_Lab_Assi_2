classdef Finger < RobotBaseClass
    properties(Access = public)   
        plyFileNameStem = 'Finger';
    end
    methods

        function self = Finger(baseTr,useTool,toolFilename)
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
    
        function CreateModel(self)

            link(1)=Link('d',0,'a',0.05715,'alpha',0,'offset',deg2rad(80),'qlim',deg2rad([0 60]));
            link(2)=Link('d',0,'a',0.0381,'alpha',0,'offset',deg2rad(0),'qlim',deg2rad([0 90]));
            link(3)=Link('d',0,'a',0.02,'alpha',0,'offset',deg2rad(10),'qlim',deg2rad([0 30]));
         
            self.model = SerialLink(link,'name',self.name);
    
        end
    end
end



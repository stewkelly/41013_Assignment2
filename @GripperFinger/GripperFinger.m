classdef GripperFinger < RobotBaseClass


    properties(Access = public)              
        plyFileNameStem = 'GripperFinger';
    end
    
    methods
%% Constructor
function self = GripperFinger(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);			
            end
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)   
            link(1) = Link('d',0,'a',0.05,'alpha',0,'qlim',deg2rad([0 20]),'offset',deg2rad(30));   
            link(2) = Link('d',0,'a',0.05,'alpha',0,'qlim',deg2rad([0 12]),'offset',deg2rad(30));
            self.model = SerialLink(link,'name',self.name);
        end    
    end
end
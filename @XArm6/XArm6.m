classdef XArm6 < RobotBaseClass
   %%
    properties(Access = public)              
        plyFileNameStem = 'XArm6';
    end
    
    methods
         
        %% Define robot Function 
        function self = XArm6(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();         
        end

        %% Create the robot model
        function CreateModel(self)   
            % Create the Xarm6 model 
           
            link(1) = Link('d',0.267,'a',0,'alpha',-pi/2,'qlim',[-2*pi, 2*pi], 'offset',0);
            link(2) = Link('d',0,'a',0.2895,'alpha',0,'qlim',[-deg2rad(118), deg2rad(120)], 'offset',deg2rad(-79));
            link(3) = Link('d',0,'a',0.0775,'alpha',-pi/2,'qlim',[-deg2rad(225), deg2rad(11)], 'offset',deg2rad(79));
            link(4) = Link('d',0.3425,'a',0,'alpha',pi/2,'qlim',[-2*pi, 2*pi], 'offset',0);
            link(5) = Link('d',0,'a',0.076,'alpha',-pi/2,'qlim',[-deg2rad(200), deg2rad(200)], 'offset',0);
            link(6) = Link('d',0.097,'a',0,'alpha',0,'qlim',[-2*pi, 2*pi], 'offset',0);

      
            self.model = SerialLink(link,'name',self.name);
        end
    end
end


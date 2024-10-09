classdef XArm6 < RobotBaseClass
   %%
    properties(Access = public)              
        plyFileNameStem = 'Xarm6';
    end
    
    methods
         
        %% Define robot Function 
        function self = XArm6(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T;
            
            
            % Call the overridden PlotAndColourRobot function
            self.PlotAndColourRobot();         
        end

        %% Create the robot model
        function CreateModel(self)   
            % Create the Xarm6 model mounted on a linear rail
           
            link(1) = Link('d',0.267,'a',0,'alpha',0,'qlim',[2*pi, 2*pi], 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',[2*pi, 2*pi], 'offset',pi);
            link(3) = Link('d',0.2805,'a',-0.051,'alpha',0,'qlim',[2*pi, 2*pi], 'offset',0);
            link(4) = Link('d',-0.3425,'a',0.0775,'alpha',pi,'qlim',[2*pi, 2*pi], 'offset',pi);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[2*pi, 2*pi], 'offset',0);
            link(6) = Link('d',0.1,'a',0.1,'alpha',pi/2,'qlim',[2*pi, 2*pi], 'offset',pi/2);

      
            self.model = SerialLink(link,'name',self.name);
        end
     
        %% Override PlotAndColourRobot to assign red color
        function PlotAndColourRobot(self)
            % Call the base class method to load .ply data
            PlotAndColourRobot@RobotBaseClass(self);

            % Set all vertices to red
            for linkIndex = 0:self.model.n
                % Set the default colour to red (R=1, G=0, B=0)
                vertexColours = repmat([0.5, 0.5, 0.5], size(self.model.points{linkIndex+1}, 1), 1); % Red color
                
                % Assign the red colour to the plot
                h = findobj('Tag', self.model.name);
                h = get(h, 'UserData');
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp'; % Interpolated shading
            end
            drawnow();
        end
    end
end


%{

 link(1) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(2) = Link('d',0.267,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(3) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',0, 'offset',deg2rad(-79.34995));
            link(4) = Link('d',0,'a',0.28948866,'alpha',0,'qlim',0, 'offset',deg2rad(79.34995));
            link(5) = Link('d',0.3425,'a',0.0775,'alpha',-pi/2,'qlim',0, 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',pi/2,'qlim',0, 'offset',0);
            link(7) = Link('d',0.097,'a',0.076,'alpha',-pi\2,'qlim',0, 'offset',0);
      

link(1) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(3) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);
            link(7) = Link('d',0,'a',0,'alpha',0,'qlim',0, 'offset',0);

%}

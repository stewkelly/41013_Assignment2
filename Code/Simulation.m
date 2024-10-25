%% Need to do
 % I've written a list of todo throughout the control loop. 
 % It is mostly done just need to tweak the animations and update some of the positions to make it look better
 % If you feel like doing the collision avoidance, feel free 
 
 % Most of the tweaks to positions will be in this class but you may have to edit some of the movement 
 % and animation functions in the robot class to get it smoother 

 % > Need to add the knife.ply to the robot for buttering the bread
 % > GUI for the simulation
 % > Collision avoidance
 % > E Stop button functionality

 % I tried to comment most of it but if you have any questions about code, just message me

 % Changes 24/10/24: 
       % Iterates through bread pieces
       % Bread stacks on final plate
       % Rotation implemented - could simplify position to be derived from 4x4 rotation
            % matrix rather than from seperate variable
       % Bread now follows gripper
       % Bread changes state correctly
       % Added toaster pop animation 
       % XArm drops bread for toasting + spreading to be completed
       % Added Gui - need impliment eStop functionality but buttons work


classdef Simulation < handle
    properties
        r1;          % First Robot instance (XArm6)
        r2;          % Second Robot instance (DobotMagician)
        breads;      % Array of BreadObject instances
        stack;       % For incrimenting stake height on plate
        guiApp;
        knife;
    end

    methods
        %% Constructor
        function self = Simulation()
            clf;
            close all;
            
            self.guiApp = gui;
            self.guiApp.setSimulationInstance(self); % Set the simulation instance

            figure('Name', 'Simulation');
            self.stack = 1.05;
            self.plotEnvironment();
            self.plotRobots();
            self.plotBread();
            self.runSim();
            
           
        end

        %% Plot Environment
        function plotEnvironment(self)
            hold on;
            axis equal;

            % Place non-bread objects in the environment
            PlaceObject('table.ply', [0, 0, 0]);
            PlaceObject('table.ply', [0, 1.5, 0]);
            PlaceObject('table.ply', [2, 0, 0]);
            PlaceObject('person.ply', [0, -1, 0]);
            PlaceObject('person.ply', [2, -1, 0]);
            PlaceObject('emergencyStopButton.ply', [2.5, 0, 1]);
            PlaceObject('emergencyStopButton.ply', [1.5, 0, 1]);
            PlaceObject('fenceAssembly.ply', [0.05, 2.1, -0.5]);
            PlaceObject('plate.ply', [0, 0.45, 1.1]);
            PlaceObject('plate.ply', [0, 1.5, 1.1]);
            PlaceObject('plate.ply', [0.5, 1, 1.1]);
            PlaceObject('toaster.ply', [-0.5, 1, 1]);
        

            % Floor
            tileTexture = imread('tile.jpg');
            surf([-1.5, 3.1; -1.5, 3.1], ...
                [-2, -2; 5, 5], ...
                [0.01, 0.01; 0.01, 0.01], ...
                'CData', tileTexture, ...
                'FaceColor', 'texturemap');


            % Left Wall
            surf([-1.5, -1.5; -1.5, -1.5], ...
                [-2, 5; -2, 5], ...
                [0, 0; 2, 2], ...
                'CData', tileTexture, ...
                'FaceColor', 'texturemap');


            % Back Wall
            surf([-1.5, 3.1; -1.5, 3.1], ...
                [5, 5; 5, 5], ...
                [0, 0; 2, 2], ...
                'CData', tileTexture, ...
                'FaceColor', 'texturemap');
        end
        
        %% Plot Robots
        function plotRobots(self)
                % Initialize XArm6 Robot
                baseR1 = transl(0, 1, 1); 
                xarm6Model = XArm6(baseR1);
                self.r1 = Robot(xarm6Model);

                % Initialize Dobot Magician Robot
                baseR2 = transl(0.5, 1.5, 1); 
                dobotModel = DobotMagician(baseR2);
                self.r2 = Robot(dobotModel);

                % Initialize joint positions (assuming zero positions)
                q1 = zeros(1, self.r1.robotModel.model.n);
                q2 = zeros(1, self.r2.robotModel.model.n);

                % Animate Robots to initial positions
                self.r1.robotModel.model.animate(q1);
                self.r2.robotModel.model.animate(q2);
                self.r1.currentPos = q1;
                self.r2.currentPos = q2;
                
               % dobotEnd = self.r2.robotModel.model.fkine(q2).t;
               % PlaceObject('knife.ply', dobotEnd');
                %hold on;
            
        end

        %% Plot Bread Objects
        function plotBread(self)
            breadPositions = [
                0, 1.5, 1.2;
                0, 1.5, 1.15;
                0, 1.5, 1.10;
                0, 1.5, 1.05
                ];
            
            % Initialises rotation matrix
            poses = eye(4);


            self.breads = cell(length(breadPositions), 1);

            % Just working with 1 bread object for now to fix bugs

            for i = 1:length(breadPositions)
                 pos = breadPositions(i, :);
                 disp("BREAD POSITION") % DEBUGGING
                 disp(pos); % DEBUGGING
                 self.breads{i} = BreadObject(pos, poses, 'bread.ply');

             end
            %hold on;
            q2 = zeros(1, self.r2.robotModel.model.n);
            dobotEnd = self.r2.robotModel.model.fkine(q2).t;
            %dobotEnd(2) = dobotEnd(2) - 0.4;
            %disp("DOBOT END POSITION") % DEBUGGING
            %     disp(dobotEnd'); % DEBUGGING
            self.knife = BreadObject(dobotEnd', poses, 'knife.ply');
            %knifeObject = BreadObject([1, 1, 1], poses, 'knife.ply');
           % pos = breadPositions(1, :);
           % self.breads{1} = BreadObject(pos, 'bread.ply');
        end

        %% Gui Buttons
        function handleEStop(self)
        % Logic to handle emergency stop
        disp('Emergency Stop Triggered');
        % Implement stopping the simulation
        end
    
        function handleReady(self)
        % Logic to handle system ready
        disp('System Ready');
        % Implement any necessary preparation for simulation
         end
    
        function handleStart(self)
        % Logic to handle starting/resuming simulation
        disp('Simulation Started');
        % Implement starting or resuming the simulation
        self.runSim();
        end


        %% Run Simulation
        function runSim(self)
            % Main Control Loop
            for i = 1:length(self.breads)
                % Current bread
                currentBread = self.breads{i}; % Right now just using 1 bread for testing
                knifeObject = self.knife;
                self.r2.pickUp(knifeObject);
                % Pick up bread and wait
                self.r1.pickUp(currentBread);
                pause(0.5);

                % Move to toaster (need to fix toaster position and orientation)
                % Also need to make animation smoother
                toasterPosition = [-0.64, 1.1, 1.2];
                toasterPose = transl(toasterPosition) * trotx(pi/2);
                self.r1.moveArm(toasterPose, 50);
                pause(0.5);
        
                % Release bread to toaster
                toasterDownPosition = [-0.64, 1.1, 1.1];
                breadPose = transl(toasterDownPosition) * trotx(pi/2); % Added for rotation into toaster
                currentBread.updatePosition(breadPose);
                % self.r1.releaseObject();
                moveAwayMatrix = toasterPose;
                moveAwayMatrix(3,4) = moveAwayMatrix(3, 4) + 1;
                disp("MOVE AWAY MATRIX SHOULD BE 4X4");
                disp(moveAwayMatrix);
                self.r1.defaultPosition(moveAwayMatrix);
                pause(0.5);
                currentBread.updateStatus('toasted'); % Update bread status to toasted - this changes bread to toast
                
                breadPose = transl(toasterPosition) * trotx(pi/2); % Added for rotation into toaster
                currentBread.updatePosition(breadPose);
                % This movement is funny, need to add collision avoidance
                % or waypoint to fix
                disp("CURRENT BREAD POSE:") % DEBUGGING
                disp(currentBread);
                self.r1.pickUp(currentBread);
                butterPosition = [0.5, 1, 1.1];
                butterPose = transl(butterPosition) * trotz(pi/2); % Position where bread needs to be buttered - need to fix
                disp("butterMoment"); % DEBUGGING
                self.r1.moveArm(butterPose, 50);
                pause(0.5);

                % Probably need to fix this too but havent got here yet
                breadPose = transl(0.5, 1, 1.1) * trotz(pi/2); % Added for rotation onto plate
                currentBread.updatePosition(breadPose);
                moveAwayMatrix = butterPose;
                moveAwayMatrix(3, 4) = moveAwayMatrix(3, 4) + 0.5;
                self.r1.defaultPosition(moveAwayMatrix);
                disp("BUTTER POSE:"); % DEBUGGING
                disp(butterPose); % DEBUGGING
                self.r2.butterBread(currentBread, butterPose, 50);
                % currentBread.updateStatus('buttered');
                pause(0.5);

                % Pick up bread and wait
                self.r1.pickUp(currentBread);
                pause(0.5);
                
                finalPosition = [0, 0.45, self.stack];
                finalPose = transl(finalPosition) * trotz(pi/2);
                self.r1.moveArm(finalPose, 50);
                currentBread.updatePosition(finalPose);
                self.stack = self.stack + 0.05;
                self.r1.releaseObject();
                pause(0.5);

            end
        end
    end
end

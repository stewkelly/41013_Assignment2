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
% Implimented Resolve Motion Rate Control
% Colision added and approach angles need tweeking for accurate movement
% Knife added but picks up inside arm of dobot
% Needs to be rotated + make it face toast consistantly
% Added collision detection (Robot class) with object vector (Simulation Class)
% Needs route recalculation - currently just outputs that there has been a collision
% EStop Currently uses uiwait which is against the assessment guide
% Need to change this but works well enough for video
% eStop functionality added within confines of assessment criteria
% Passes status from timer callback to Robot, if statment during movement process to stop motion
% Arduino eStop added - only works on first run due to port definition bug

% 29/10 Changes -------------
% > Updated RMRC control in robot class
% > Cleaned up Robot class
% > Changed environment slightly by moving robots and objects for better movement 
% > Changed steps in main loop so buttering now happens over toaster
% > Smoothed out main loop to avoid collisions
% > Created CollisionDetection class - kinda works but commented the
% avoidance out and changed to stop cause it freezes sometimes


classdef Simulation < handle
    properties
        
        r1;          % First Robot instance (XArm6)
        r2;          % Second Robot instance (DobotMagician)
        breads;      % Array of BreadObject instances
        stack;       % For incrimenting stake height on plate
        home;
        guiApp;
        knife;
        table;
        table2;
        table3;
        plate;
        plate2;
        plate3;
        fence;
        toaster;
        pointCloudVector;
        status = 'running';
        eStopActive = false;  % Track e-stop state
        eStopTimer;           % Timer object
        arduinoObj;   % Arduino object for serial communication
        arduinoTimer;
        serialObj;
    end

    events
        eStopTriggered;
        readyTriggered;
        startTriggered;
    end

    methods
        %% Constructor
        function self = Simulation()
            clf;
            close all;

            self.guiApp = gui;
            self.guiApp.setSimulationInstance(self); % Set the simulation instance

            self.eStopTimer = timer('ExecutionMode', 'fixedRate', ...
                                    'Period', 0.5, ...  % Check every 100ms
                                    'TimerFcn', @(~,~)self.checkEStop);
            
   
       %-----------------Comment out to remove arduino--------------------
            %self.arduinoObj = arduino;
            %configurePin(self.arduinoObj, "D2", "Pullup");
            %configurePin(self.arduinoObj, "D3", "Pullup");
            %configurePin(self.arduinoObj, "D4", "Pullup");
            %cleanupObj = onCleanup(@() self.cleanupArduino);
            %self.arduinoTimer = timer('ExecutionMode', 'fixedRate', ...
            %                  'Period', 0.5, ... % Adjust interval as needed
            %                  'TimerFcn', @(~,~)self.readArduinoData);

       %-----------------------------------------------------------------      
           
            % Event listeners
            addlistener(self, 'eStopTriggered', @(~, ~) self.handleEStop());
            addlistener(self, 'readyTriggered', @(~, ~) self.handleReady());
            addlistener(self, 'startTriggered', @(~, ~) self.handleStart());

            figure('Name', 'Simulation');
            self.stack = 1.05;
            self.plotEnvironment();
            self.plotRobots();
            self.plotBread();
            self.r1.collisionDetector.setObstacles(self.pointCloudVector);

            self.runSim();
        end
        
        %% Cleanup on Exit for Arduino
        function cleanupArduino(self)
            stop(self.arduinoTimer);
            delete(instrfind);
            clear arduino;
            clear aurduino_object;
            %delete(self.arduinoObj);
            clear self.arduinoObj;
            clear arduinoObj;
            clear arduino;
            clear all;
            % Clear any existing Arduino objects from the workspace
            disp('Program is exiting. Clearing Arduino object.');
        end

        %% Point Clouds for Collisions
        function pointCloudGlobal = loadPointCloud(self, plyFile, coordinates)
            % Original place function still used
            PlaceObject(plyFile, coordinates);

            % Load PLY as a point cloud object
            pointCloudLocal = pcread(plyFile);

            % Extract points from point cloud object
            pointsLocal = pointCloudLocal.Location;

            % Define transformation from object frame to global frame
            objectToWorld = transl(coordinates); % Transformation matrix (adjust based on your data)

            % Transform points to global frame
            pointsGlobal = (objectToWorld * [pointsLocal, ones(size(pointsLocal, 1), 1)]')'; % Add homogenous coordinates

            % Remove homogenous coordinate
            pointCloudGlobal = pointsGlobal(:, 1:3);
        end
      
        %% Plot Environment
        function plotEnvironment(self)
            hold on;
            axis equal;

            % Place non-bread objects in the environment
            self.table = self.loadPointCloud('table.ply', [0, 0, 0]);
            self.table2 = self.loadPointCloud('table.ply', [0, 1.5, 0]);
            self.table3 = self.loadPointCloud('table.ply', [2, 0, 0]);
            PlaceObject('person.ply', [0, -1, 0]);
            PlaceObject('person.ply', [2, -1, 0]);
            PlaceObject('emergencyStopButton.ply', [2.5, 0, 1]);
            PlaceObject('emergencyStopButton.ply', [1.5, 0, 1]);
            self.fence = self.loadPointCloud('fenceAssembly.ply', [0.05, 2.1, -0.5]);
            self.plate = self.loadPointCloud('plate.ply', [-0.07, 0.5, 1.05]);
            self.plate2 = self.loadPointCloud('plate.ply', [0, 1.4, 1.05]);
            self.toaster = self.loadPointCloud('toaster.ply', [-0.5, 1, 1]);

            % Stores objects in vector - probably a cleaner way to do this
            self.pointCloudVector = {self.table, self.table2, self.table3,...
                self.fence, self.plate, self.plate2, self.toaster};

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
            baseR2 = transl(-0.5, 1.4, 1);
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

            self.home = self.r1.robotModel.model.fkine(q1).T;
        end

        %% Plot Bread Objects
        function plotBread(self)
            breadPositions = [
                0, 1.4, 1.2;
                0, 1.4, 1.15;
                0, 1.4, 1.10;
                0, 1.4, 1.05
                ];

            % Initialises rotation matrix
            poses = eye(4);
            self.breads = cell(length(breadPositions), 1);

            for i = 1:length(breadPositions)
                pos = breadPositions(i, :);
                self.breads{i} = BreadObject(pos, poses, 'bread.ply');
            end



            %hold on;
            %q2 = zeros(1, self.r2.robotModel.model.n);
            %dobotEnd = self.r2.robotModel.model.fkine(q2).t;

            %dobotEnd(2) = dobotEnd(2) - 0.4;
            %disp("DOBOT END POSITION") % DEBUGGING
            %     disp(dobotEnd'); % DEBUGGING


            % Comment out to get rid of knife
            % ----------------------------------------

            % self.knife = BreadObject([0.8, 1.3, 1.1], knifePose, 'knife.ply');
            % knifePose = transl(0.8, 1.3, 1.1) * trotz(pi);

            % ----------------------------------------

            % pos = breadPositions(1, :);
            % self.breads{1} = BreadObject(pos, 'bread.ply');

        end

        %% Check eStop
        function checkEStop(self)
            if strcmp(self.status, 'stopped')  || strcmp(self.r1.status, 'stopped')
                disp('Emergency Stop Active!');
                notify(self, 'eStopTriggered');  % Trigger the event
                stop(self.eStopTimer);  % Stop the timer to halt execution
                %stop(self.arduinoTimer)
            end
            self.r1.timerStatusCheck(self.status);
            self.r2.timerStatusCheck(self.status);
        end

        %% Arduino Callbacks
        function readArduinoData(self)
            disp("READING ARDUINO");
        
            % Read digital pin states directly
            eStopState = readDigitalPin(self.arduinoObj, "D2");
            readyState = readDigitalPin(self.arduinoObj, "D3");
            startState = readDigitalPin(self.arduinoObj, "D4");
        
            disp(['eStop: ', num2str(eStopState), ' Ready: ', num2str(readyState), ' Start: ', num2str(startState)]); % Debugging line
        
            if eStopState == 1
                self.stopButtonCallback();
            elseif readyState == 1
                self.readyButtonCallback();
            elseif startState == 1
                self.startButtonCallback();
            end
        end



        %% Interrupt Callbacks
        function stopButtonCallback(self)
            notify(self, 'eStopTriggered');  % Trigger eStop event
        end

        function readyButtonCallback(self)
            notify(self, 'readyTriggered');  % Trigger ready event
        end

        function startButtonCallback(self)
            notify(self, 'startTriggered');  % Trigger start event
        end


        %% Gui Buttons
        function handleEStop(self)
            disp('Emergency Stop Triggered');
            self.status = 'stopped';
        end

        function handleReady(self)
            disp('System Ready');
            self.status = 'ready';
        end

        function handleStart(self)
            if strcmp(self.status, 'ready')
                disp('Simulation Started');
                self.status = 'running';
                start(self.eStopTimer);
                %start(self.arduinoTimer);
            else
                disp('System not ready. Press "Ready" first.');
            end
        end



        %% Run Simulation
        function runSim(self)
            % Passes obsticals to Robot platforms
            start(self.eStopTimer);
            %start(self.arduinoTimer);
            %self.r1.obsticalVectorCallback(self.pointCloudVector);
            %self.r2.obsticalVectorCallback(self.pointCloudVector);
            % Main Control Loop
            for i = 1:length(self.breads)
                %if strcmp(self.status, 'stopped')
                %    uiwait();  % Wait until uiresume() is called
                %end
                % Current bread
               
                % ----------------------------------------
                % Comment out to get rid of knife
                % knifeObject = self.knife;
                % self.r2.pickUp(knifeObject);
                % defaultButterPose = transl(0.4400, 1.5000, 1.4712) * trotx(pi/2);
                % self.r2.moveArm(defaultButterPose, 50);

                % ---------------------------------- 29/10 Changes
                currentBread = self.breads{i}; 

                %% Step 1: Pick up Bread
                % Move Arm to Bread
                % Close Gripper
                % Update Bread Position to EE Position
                approachPose = transl(currentBread.position) * transl([0.2,0,0]) * troty(-pi/2); % To approach from the right side
                self.r1.moveRMRC(approachPose);
                self.r1.closeGripper(currentBread);
                pause(0.5);

                %% Step 2: Move to Toaster
                % Move Arm to Toaster
                % Gripper Closed
                % Bread Position needs to update with EE Position
                toasterPose = transl([-0.5, 1.1, 1.4]) * trotx(pi) * trotz(pi/2); % To approach from top
                self.r1.moveRMRC(toasterPose);

                %% Step 3: Release Bread to Toast
                % Open Gripper
                % Drop Bread
                % Bread changes to toast in same position
                % Toast rises
                self.r1.openGripper();
                breadPose = toasterPose * transl([0,0,0.1]);
                currentBread.updatePosition(breadPose);
                self.r1.moveRMRC(toasterPose * transl([0,0,-0.12]));
                currentBread.updateStatus('toasted'); % Update bread status to toasted - this changes bread to toast
                % Add the toast point cloud to the collision detection vector
                self.pointCloudVector{end+1} = currentBread.pointCloud;
                self.r1.collisionDetector.setObstacles(self.pointCloudVector); 

                %% Step 4: Grab Toast
                % Close Gripper
                self.r1.closeGripper(currentBread);


                %% Step 5: Butter Toast
                disp('Starting to butter the toast...');

                butteringPose = transl([-0.5, 1, 1.25]);
                self.r2.butterBread(butteringPose);
                currentBread.updateStatus('buttered'); % Update toast to toastButtered
                pause(1.0); 

                self.pointCloudVector{end+1} = currentBread.pointCloud;
                self.r1.collisionDetector.setObstacles(self.pointCloudVector); % Update point cloud
                disp('Buttering completed.');


                %% Step 6: Move Buttered Toast to Final Plate
                % Final plate pose
                finalPlatePose = transl([-0.4, 0.5, self.stack]) * troty(pi/2); 
                self.r1.moveRMRC(finalPlatePose);
                self.stack = self.stack + 0.05;
                pause(1.0);

                %% Step 7: Release Buttered Toast and Go Home
                self.r1.openGripper();
                currentBread.updatePosition(finalPlatePose * transl([0,0,0.2])); 
                currentBread.updateStatus('completed'); 
                disp('Buttered toast placed on the final plate.');
                pause(1.0);
                self.r1.moveArm(self.home);
            end
        end
    end
end

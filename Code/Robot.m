classdef Robot < handle
    properties
        robotModel     % Robot model
        gripper        % Gripper instance
        currentPos     % Current joint positions
        holdingObject
        objectAvoidVector
        status
        
    end

    methods
        %% Constructor
        function self = Robot(robotModel)
            if nargin < 2
                baseTr = eye(4);
            end
            self.robotModel = robotModel;
            self.currentPos = self.robotModel.model.getpos();
            gripperTr = self.robotModel.model.fkine(self.currentPos).T;
            self.gripper = Gripper(gripperTr);
            self.holdingObject = [];
            self.objectAvoidVector = [];
            self.status = 'running';
        end
        
        %% Accepts and Save Obstical Vector from Simulation
        function obsticalVectorCallback(self, pointCloudVector)
            self.objectAvoidVector = pointCloudVector;
            disp("IN ROBOT OBJECT AVOID VECTOR:");
            disp(length(self.objectAvoidVector));
        end
        
        %% Timer Callback
        function timerStatusCheck(self, timerStatus)
            self.status = timerStatus; 
        end
        

        %% Check Status
        function secondStatus = updateSecondStatus(self)
            if strcmp(self.status, 'stopped')
               secondStatus = 0;
            else
               secondStatus = 1;
            end
        end

        %% Collision Check
        function isCollision = checkEllipsoidCollision(self, currentJoints)
            % Extract points from point cloud
            
            % Define sphere around end effector - determines sensitivity
            radii = [0.3, 0.2, 0.4];
            % Define end effector
            eePose = self.robotModel.model.fkine(currentJoints).T;
            
            % Iterate through objects
            for j = 1:size(self.objectAvoidVector, 1)+1
            % Extract points from point cloud
            points = self.objectAvoidVector{j};
                % Iterate through points in point cloud
                for i = 1:size(points, 1)
            
                    % Compare point location to end effector
                    shiftedPoints = points(i, :) - eePose(1:3, 4)';
                    % Calculate normalized squared distances based on elipsiod
                    normalizedDistances = (shiftedPoints(:, 1) / radii(1)).^2 + ...
                                          (shiftedPoints(:, 2) / radii(2)).^2 + ...
                                          (shiftedPoints(:, 3) / radii(3)).^2;
                
                    % Check if any point lies within the ellipsoid
                    isCollision = any(normalizedDistances <= 1);
                       if isCollision == true % Returns bool
                            break;
                       end
                 end
             end
        end

        %% Velocity Kinematics
        function qMatrix = moveWithVelocityKinematics(self, goalPose, currentJointConfig)
            dt = 0.1;   % Time step
            lambda = 0.1; % Damping factor
            maxSteps = 100; % Safety limit for iterations
            i = 1; % MatLab starts values from 1 instead of 0 apparently
        
            while i <= maxSteps
                check = updateSecondStatus(self);
                if check == 0
                    disp('WE ARE IN THE COLLISION LOOP');
                    pause(0.1);  % Non-blocking wait loop
                    
                else
                % Calculate current end-effector pose
                currentPose = self.robotModel.model.fkine(currentJointConfig);
        
                % Compute error in Cartesian space
                error = tr2delta(currentPose, goalPose);
        
                % Define desired Cartesian velocity (e.g., proportional to error)
                v = 0.5 * error;  % Adjust scaling as needed
        
                % Compute the Jacobian matrix
                J = self.robotModel.model.jacob0(currentJointConfig);
        
                % Damped least squares solution for joint velocities
                J_dls = (J' * J + lambda^2 * eye(size(J, 2))) \ J';
                q_dot = J_dls * v;
        
                % Update joint configuration
                currentJointConfig = currentJointConfig + q_dot' * dt;
                % Adds joint configuration to next row of control matrix
                qMatrix(i, :) = currentJointConfig;
                % Iterates as amount of steps is undefined
                
                
                    i = i + 1;
                    % Collision Detection
                    collisionDetected = self.checkEllipsoidCollision(currentJointConfig);
                    
                    % Collision control loop - only detects at the moment
                        % Needs path correction implimented here
                    if collisionDetected
                        disp('Collision detected!');
                    else
                        disp('No collision.');
                    end
                
                    % Check for convergence - if EE has reached destination
                    if norm(error) < 0.01
                        break;
                    end
                end   
            end
        
            disp('Movement complete.');
        end


        %% Move Arm
        function moveArm(self, targetPose, steps)
            qNow = self.robotModel.model.getpos();
            disp("qNow:") % DEBUGGING
            disp(qNow); % DEBUGGING
            disp("targetPose:") % DEBUGGING
            disp(targetPose); % DEBUGGING
            qTarget = self.robotModel.model.ikcon(targetPose, qNow);
            
          %JTraj Movement: 
            %qMatrix = jtraj(qNow, qTarget, steps);

          %ResolveMotionRateControl Movement:
            qMatrix = self.moveWithVelocityKinematics(targetPose, qNow);
            
            self.animateMovement(qMatrix);
            self.currentPos = qTarget;
            %self.currentPos = self.robotModel.model.getpos();

        end

        %% Animate Movement
        function animateMovement(self, qMatrix)
            i = 1;
            while i <= size(qMatrix, 1)
                if strcmp(self.status, 'stopped')
                    disp('WE ARE IN THE ANIMATION LOOP');
                    pause(0.1);  % Non-blocking wait loop
                    continue;
                end
        
                self.robotModel.model.animate(qMatrix(i, :));
                eePose = self.robotModel.model.fkine(qMatrix(i, :)).T;
                self.gripper.updatePosition(eePose);
        
                if ~isempty(self.holdingObject)
                    self.holdingObject.updatePosition(eePose);
                end
        
                pause(0.01);  % Control animation speed
                i = i + 1;
            end
        end

        %% Open Gripper
        function openGripper(self, steps)
            self.gripper.openGripper(steps);
            if ~isempty(self.holdingObject)
                self.releaseObject();
            end
        end

        %% Close Gripper
        function closeGripper(self, steps, object)
            self.gripper.closeGripper(steps);
            if self.gripper.isHolding()
                self.holdingObject = object;
            end
        end

        %% Pick Up Object
        function pickUp(self, breadobject)
            % Approach the object
            approachPose = transl(breadobject.position) * trotx(pi/2);
            %approachPose = transl(breadobject.position);
            self.moveArm(approachPose, 50);
            self.closeGripper(50, breadobject);
            
        end

        %% Release Object
        function releaseObject(self)
            self.gripper.openGripper(50);
            if ~isempty(self.holdingObject)
         
        % Updates status in Simulation - this conflicts with current setup
                % Update object's status based on current state
                % currentFile = self.holdingObject.file;
                % switch currentFile
                %    case 'bread.ply'
                %       self.holdingObject.updateStatus('toasted');
                %    case 'toast.ply'
                %       self.holdingObject.updateStatus('buttered');
                %    case 'toastButtered.ply'
                % end
                
                self.holdingObject = [];
            end
        end

        %% Butter Bread
        function butterBread(self, currentBread, butterPose, steps)
            % Move to the initial butter pose
            self.moveArm(butterPose, steps);
            
            offset = 0.05; 
            butterPose1 = butterPose * transl(0, -offset, 0); 
            butterPose2 = butterPose * transl(0, offset, 0);  

            cycles = 3; % Adjust as needed for simulation

            for cycle = 1:cycles                
                self.moveArm(butterPose1, steps);
                pause(0.1); 

                self.moveArm(butterPose2, steps);
                pause(0.1); 
            end
            currentBread.updateStatus('buttered');
            butterPose(3, 4) = butterPose(3, 4) + 1;
            self.moveArm(butterPose, steps);
            

        end
        
        %% Return to default position
        function defaultPosition(self, defaultPose)
            self.releaseObject();
            self.moveArm(defaultPose, 50);
        end  

        
    end
end

classdef Robot < handle
    properties
        robotModel     % Robot model
        gripper        % Gripper instance
        currentPos     % Current joint positions
        holdingObject
        collisionDetector
        % objectAvoidVector
        status

    end

    properties(Constant)
        steps = 50;
    end

    methods
        %% Constructor
        function self = Robot(robotModel)
            if nargin < 2
                baseTr = eye(4);
            end
            self.robotModel = robotModel;
            self.collisionDetector = CollisionDetection(0.03);
            self.currentPos = self.robotModel.model.getpos();
            gripperTr = self.robotModel.model.fkine(self.currentPos).T;
            self.gripper = Gripper(gripperTr);
            self.holdingObject = [];
            % self.objectAvoidVector = [];
            self.status = 'running';
        end

        %% Velocity Kinematics
        function moveRMRC(self, goalPose)
            dt = 0.1;   % Time step
            lambda = 0.05; % Damping factor
            maxSteps = 100; % Safety limit for iterations
            i = 1; % MatLab starts values from 1 instead of 0 apparently
            K = 1.0; % Gain
            
            %  -----------------------------  29/10 Update
            qNow = self.currentPos;

            disp("Starting RMRC movement...");
            for step = i:maxSteps
                if strcmp(self.status, 'stopped')
                    disp('E Stop in Place');
                    pause(0.1);  % Non-blocking wait loop
                    continue;
                end

                currentPose = self.robotModel.model.fkine(qNow).T;
                error = tr2delta(currentPose, goalPose);

                if norm(error) < 1e-3
                    disp('Target pose reached.');
                    break;
                end

                v = K * error(1:3); % Linear
                w = K * error(4:6); % Angular

                vw = [v; w];

                % Compute the Jacobian matrix
                J = self.robotModel.model.jacobe(qNow); % Was computing in base frame before with jacob0. Now in end effector frame
                

                % Damped least squares solution for joint velocities
                if self.robotModel.model.n == 6
                    J_dls = inv((J'*J) + lambda^2*eye(6))*J';    
                else
                    J_dls = inv((J'*J) + lambda^2*eye(5))*J';
                end
                q_dot = J_dls * vw;

                % Update joint configuration
                qNext = qNow + q_dot' * dt;
                self.currentPos = qNext;  % Directly update the stored position

                % Collision Check
                if self.collisionDetector.checkCollision(qNext, self.robotModel.model)
                    disp('Collision Detected. Altering Pose.')
                    % alteredPose = self.avoidCollision(goalPose);
                    % self.moveRMRC(alteredPose);
                    return;
                end

                % Update robot model and gripper
                self.robotModel.model.animate(qNext);
                eePose = self.robotModel.model.fkine(qNext).T;
                self.gripper.updatePosition(eePose);

                % Update held object position if any
                if ~isempty(self.holdingObject)
                    self.holdingObject.updatePosition(self.robotModel.model.fkine(qNext).T);
                end

                % Update current joint positions
                qNow = qNext;

                pause(dt); % Simulate real-time movement
            end
            disp('Movement complete.');

        end

        %% Open Gripper
        function openGripper(self)
            self.gripper.openGripper(self.steps);
            self.holdingObject = [];
        end

        %% Close Gripper
        function closeGripper(self,  object)
            self.gripper.closeGripper(self.steps);
            self.holdingObject = object;
        end

        %% Butter Bread
        function butterBread(self, butterPose)
            % Move to the initial butter pose
            self.moveRMRC(butterPose);

            % offset = 0.1;
            % butterPose1 = butterPose * transl(-offset, 0, 0);
            % butterPose2 = butterPose * transl(offset, 0, 0);
            % 
            % self.moveRMRC(butterPose1);
            % pause(0.1);
            % 
            % self.moveRMRC(butterPose2);
            % pause(0.1);
            % 
            % butterPose(3, 4) = butterPose(3, 4) + 1;
            % self.moveRMRC(butterPose);
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

        %% Movement if Collision is Found
        function alteredPose = avoidCollision(self, goalPose)
            % Check for initial collision at goalPose
            if ~self.collisionDetector.checkCollision(self.currentPos, self.robotModel.model)
                alteredPose = goalPose;  % No collision, return original goalPose
                return;
            end

            % Retrieve the current end-effector pose from the robot
            currentPose = self.robotModel.model.fkine(self.currentPos).T;

            % Calculate the backward direction vector directly within the method
            directionVector = goalPose(1:3, 4) - currentPose(1:3, 4);
            directionVector = -directionVector / norm(directionVector);  % Normalize and reverse

            % Set parameters for incremental adjustments
            maxAttempts = 10;
            stepSize = 0.02;  % Step size in meters
            rotationStep = troty(-pi/20); % Adjust this as needed

            % Try moving backward and adjusting rotation
            for i = 1:maxAttempts
                % Adjust translation backward along directionVector
                adjustedPose = goalPose * transl(directionVector * stepSize * i);

                % Gradually rotate to the goal orientation to avoid passing through self
                rotationAdjust = rotationStep^i;
                adjustedPose = adjustedPose * rotationAdjust;

                % Use inverse kinematics to find a joint configuration
                adjustedConfig = self.robotModel.model.ikcon(adjustedPose, self.currentPos);

                % Check if the adjusted configuration is collision-free
                if ~self.collisionDetector.checkCollision(adjustedConfig, self.robotModel.model)
                    disp(['Collision avoided by adjusting pose on attempt ', num2str(i)]);
                    alteredPose = adjustedPose;
                    return;
                end
            end

            % If no safe pose is found, return the original goalPose
            disp('Unable to find collision-free pose; returning original pose.');
            alteredPose = goalPose;
        end

        function moveArm(self, targetPose)
            qNow = self.robotModel.model.getpos();
            qTarget = self.robotModel.model.ikcon(targetPose, qNow);
            
          %JTraj Movement: 
            qMatrix = jtraj(qNow, qTarget, self.steps);

          % %ResolveMotionRateControl Movement:
          %   qMatrix = self.moveWithVelocityKinematics(targetPose, qNow);
            
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


    end
end


       
        



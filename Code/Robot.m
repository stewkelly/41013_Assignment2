classdef Robot < handle
    properties
        robotModel     % Robot model
        gripper        % Gripper instance
        currentPos     % Current joint positions
        holdingObject  
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
        end

        %% Move Arm
        function moveArm(self, targetPose, steps)
            qNow = self.robotModel.model.getpos();
            qTarget = self.robotModel.model.ikcon(targetPose, qNow);
            qMatrix = jtraj(qNow, qTarget, steps);
            self.animateMovement(qMatrix);
            self.currentPos = qTarget;
        end

        %% Animate Movement
        function animateMovement(self, qMatrix)
            for i = 1:size(qMatrix, 1)
                self.robotModel.model.animate(qMatrix(i, :));
                eePose = self.robotModel.model.fkine(qMatrix(i, :)).T;
                self.gripper.updatePosition(eePose);
                pause(0.01);
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
            self.moveArm(approachPose, 50);
            self.closeGripper(50, breadobject);
        end

        %% Release Object
        function releaseObject(self)
            self.gripper.openGripper(50);
            if ~isempty(self.holdingObject)
                % Update object's status based on current state
                currentFile = self.holdingObject.file;
                switch currentFile
                    case 'bread.ply'
                        self.holdingObject.updateStatus('toasted');
                    case 'toast.ply'
                        self.holdingObject.updateStatus('buttered');
                    case 'toastButtered.ply'
                end
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
            self.moveArm(butterPose, steps);
        end
    end
end

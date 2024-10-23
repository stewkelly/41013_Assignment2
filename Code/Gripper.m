classdef Gripper < handle
    properties
        finger1        % First GripperFinger instance
        finger2        % Second GripperFinger instance
        holding = false; % Indicates if the gripper is holding an object
    end

    methods
        %% Constructor
        function self = Gripper(baseTr)
            % Constructor for the Gripper class
            if nargin < 1
                baseTr = eye(4); % Default transformation
            end

            % Initialize the two gripper fingers 
            self.finger1 = GripperFinger(baseTr * trotx(pi/2));
            self.finger2 = GripperFinger(baseTr * trotx(pi/2) * troty(pi));
        end

        %% Open Gripper
        function openGripper(self, steps)
            qOpen = [deg2rad(10), deg2rad(0)];
            qNow = self.finger1.model.getpos();
            qMatrix = jtraj(qNow, qOpen, steps);

            self.animateMovement(qMatrix);
            self.holding = false;
        end

        %% Close Gripper
        function closeGripper(self, steps)
            qClose = [deg2rad(20), deg2rad(12)];
            qNow = self.finger1.model.getpos();
            qMatrix = jtraj(qNow, qClose, steps);

            self.animateMovement(qMatrix);
            self.holding = true;
        end

        %% Animate Movement
        function animateMovement(self, qMatrix)
            % Animates both gripper fingers simultaneously
            for i = 1:size(qMatrix, 1)
                self.finger1.model.animate(qMatrix(i, :));
                self.finger2.model.animate(qMatrix(i, :));
                pause(0.01); 
            end
        end

        %% isHolding
        function holding = isHolding(self)
            holding = self.holding;
        end

        function updatePosition(self, baseTr)
            self.finger1.model.base = baseTr;
            self.finger2.model.base = baseTr;
        end
    end
end

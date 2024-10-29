classdef CollisionDetection < handle
    properties
        objectAvoidVector    % Cell array of point clouds for obstacles
        collisionThreshold   % Distance threshold for collision (in meters)
    end

    methods
        %% Constructor
        function self = CollisionDetection(collisionThreshold)
            if nargin < 1
                self.collisionThreshold = 0.05;
            else
                self.collisionThreshold = collisionThreshold;
            end
            self.objectAvoidVector = {};
        end

        %% Set Obstacles
        function setObstacles(self, pointCloudVector)
            % Sets the obstacle point clouds
            self.objectAvoidVector = pointCloudVector;
            disp(['Obstacles set with ', num2str(length(pointCloudVector)), ' point clouds.']);
        end

        %% Check Collision
        function isCollision = checkCollision(self, jointConfig, robotModel)
            % Checks for collision around each joint using ellipsoids
            isCollision = false;
            jointPositions = self.getJointPositions(jointConfig, robotModel);
            numJoints = size(jointPositions, 1);

            % Loop through each joint position to check for collision
            for joint = 1:numJoints
                jointPosition = jointPositions(joint, :); % Get current joint position

                % Check for collision around this joint using an ellipsoid
                if self.checkEllipsoidCollision(jointPosition)
                    isCollision = true;
                    disp(['Collision detected at joint ', num2str(joint)]);
                    return; % Early exit on collision
                end
            end
        end

        %% Get Joint Positions
        function jointPositions = getJointPositions(self, jointConfig, robotModel)
            % Returns the positions of all joints based on the current joint configuration
            numJoints = robotModel.n;
            jointPositions = zeros(numJoints + 1, 3); % +1 for the end effector

            for i = 1:numJoints
                % Create a full joint configuration vector
                q = zeros(1, numJoints);
                q(1:i) = jointConfig(1:i); % Set the first i joints
                % The remaining joints are set to zero

                % Compute the forward kinematics for the first i joints
                T = robotModel.fkine(q);
                jointPositions(i, :) = T.t'; % Store the position of the current joint
            end
            % Compute the forward kinematics for the full joint configuration
            T = robotModel.fkine(jointConfig);
            jointPositions(end, :) = T.t'; % Store the position of the end effector
        end

        %% Check Collision for Ellipsoid
        function collision = checkEllipsoidCollision(self, jointPosition)
            % Define radii 
            radii = [0.01, 0.01, 0.01]; 

            collision = false;
            invRadiiSquared = 1 ./ (radii .^ 2); % Precompute inverse squared radii for efficiency

            % Loop through each obstacle in the environment
            for obj = 1:length(self.objectAvoidVector)
                objectPoints = self.objectAvoidVector{obj}; 

                % Translate object points to the ellipsoid's local frame
                translatedPoints = objectPoints - jointPosition;

                % Calculate scaled distances squared for each point
                distancesSquared = sum((translatedPoints .^ 2) .* invRadiiSquared, 2);

                % Check if any point lies within the ellipsoid
                if any(distancesSquared <= 1)
                    collision = true;
                    return; 
                end
            end
        end
    end
end

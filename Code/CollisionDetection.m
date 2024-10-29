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
            % Checks for collision across all robot links
            isCollision = false;
            jointPositions = self.getJointPositions(jointConfig, robotModel);
            numLinks = size(jointPositions, 1) - 1;
            
            for link = 1:numLinks
                p1 = jointPositions(link, :);
                p2 = jointPositions(link + 1, :);
                if self.checkLinkCollision(p1, p2)
                    isCollision = true;
                    disp(['Collision detected on link ', num2str(link)]);
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
        
        %% Check Collision for a Single Link
        function collision = checkLinkCollision(self, p1, p2)
            % Checks if the line segment between p1 and p2 is within threshold distance to any object point
            collision = false;
            for obj = 1:length(self.objectAvoidVector)
                objectPoints = self.objectAvoidVector{obj};
                % Calculate the minimum distance from all object points to the link
                distances = self.pointToSegmentDistance(objectPoints, p1, p2);
                if any(distances <= self.collisionThreshold)
                    collision = true;
                    return;
                end
            end
        end
        
        %% Helper Function: Point to Segment Distance
        function distances = pointToSegmentDistance(~, points, p1, p2)
            % Calculates the minimum distance from each point to the line segment defined by p1 and p2
            % points: [N x 3] matrix
            % p1, p2: [1 x 3] vectors defining the line segment
            
            % Vector from p1 to p2
            segVec = p2 - p1;
            segLen = norm(segVec);
            if segLen == 0
                distances = vecnorm(points - p1, 2, 2);
                return;
            end
            segUnit = segVec / segLen;
            
            % Vector from p1 to each point
            p1ToPoints = points - p1;
            
            % Project each point onto the segment, computing parameter t
            t = p1ToPoints * segUnit';
            
            % Clamp t to the [0, segLen] range
            t(t < 0) = 0;
            t(t > segLen) = segLen;
            
            % Closest points on the segment
            closestPoints = p1 + t .* segUnit;
            
            % Compute distances
            differences = points - closestPoints;
            distances = vecnorm(differences, 2, 2);
        end
    end
end

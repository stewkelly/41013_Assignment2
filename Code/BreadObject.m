classdef BreadObject < handle
    properties
        position;   % Object's [x,y,z] position
        pose;       % Object's [yaw, pitch, roll, position] pose
        file;       % Object's .ply file
        mesh;       % Object's trisurf object
    end

    methods
        function self = BreadObject(position, pose, file)
            self.position = position;
            self.pose = pose; % For rotation
            self.file = file;
            self.PlotObject();
        end

        function PlotObject(self)
            % Load the object model and plot it at the given position
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Plot the object at the initial position and store the mesh handle
            R = self.pose(1:3, 1:3);  % Extract the rotation matrix from the pose
            rotatedVertices = (R * vertices')';

            % Shift vertices based on the initial position
            shiftedVertices = rotatedVertices + self.position;

            % Plot the object at the initial position and store the mesh handle
            self.mesh = trisurf(faces, ...
                shiftedVertices(:, 1), ...
                shiftedVertices(:, 2), ...
                shiftedVertices(:, 3), ...
                'FaceVertexCData', vertexColors, ...
                'EdgeColor', 'none', ...
                'EdgeLighting', 'none');
        end

        function updatePosition(self, newPosition, newPose)
            % Update the object's position and move its mesh accordingly
            self.position = newPosition;
            self.pose = newPose;
        
            % Load the vertices and colors from the PLY file
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        
            % Changed how verticies are shifted - added extra step to account
            % for rotation - appears in changeFile as well

            % Extract rotation and translation from the new pose
            R = newPose(1:3, 1:3);  % Rotation matrix
            
            % Check matrix dimension - should be its own function
            if ~isequal(size(newPosition), [1, 3])  % Check if matrix is not 3x1
            if numel(newPosition) == 3  % If it contains 3 elements but in wrong shape
                newPosition = reshape(newPosition, [1, 3]);  % Reshape to 3x1
            else
                error('Input matrix must contain exactly 3 elements for translation.');
            end
            end

            % Apply rotation and translation to each vertex
            transformedVertices = (R * vertices')' + newPosition;
        
            % Ensure vertices and faces are updated properly
            try
                set(self.mesh, 'Faces', faces, ...
                    'Vertices', transformedVertices, ...
                    'FaceVertexCData', vertexColors);
                drawnow;
                disp(['Bread object moved to new position: ', mat2str(newPosition)]);
            catch ME
                disp('Error updating bread object position:');
                disp(ME.message);
            end
        end


        function releaseObject(self)
            % Optional: Remove the mesh if the object is released permanently
            delete(self.mesh);
            self.mesh = [];
            disp('Bread object has been removed from the environment.');
        end


        function changeFile(self, newFile)
            % Change the PLY file associated with the object and update the mesh
            self.file = newFile;
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Changed how verticies are shifted - added extra step to account
        % for rotation - appears in updatePosition as well

            % Extract the rotation matrix and translation vector from the current pose
            R = self.pose(1:3, 1:3);  % Rotation matrix (3x3)
            T = self.position;   % Translation vector (1x3)
            
            % Check matrix dimension - should be its own function
            if ~isequal(size(T), [1, 3])  % Check if matrix is not 3x1
            if numel(T) == 3  % If it contains 3 elements but in wrong shape
                T = reshape(T, [1, 3]);  % Reshape to 3x1
            else
                error('Input matrix must contain exactly 3 elements for translation.');
            end
            end

            % Apply the current pose: Rotate vertices, then translate
            rotatedVertices = (R * vertices')';  % Rotate vertices
            transformedVertices = rotatedVertices + T;  % Translate vertices

            % Ensure shiftedVertices is [N x 3]
            if size(transformedVertices, 2) ~= 3
                error('Shifted vertices must have three columns (x, y, z).');
            end

            % Update mesh vertices and faces
            set(self.mesh, 'Faces', faces, ...
                'Vertices', transformedVertices, ...
                'FaceVertexCData', vertexColors);
            drawnow;
        end


        function updateStatus(self, status)
            % Update the object's status and change its PLY file accordingly
            switch status
                case 'toasted'
                    self.changeFile('toast.ply');
                    disp('Bread has been toasted.');
                case 'buttered'
                    self.changeFile('toastButtered.ply');
                    disp('Toast has been buttered.');
                case 'released'
                    disp('Object has been released.');
                otherwise
                    disp(['Unknown status: ', status]);
            end
        end
    end
end
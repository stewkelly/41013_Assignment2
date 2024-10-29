classdef BreadObject < handle
    properties
        position;   % Object's [x,y,z] position
        pose;       % Object's [yaw, pitch, roll, position] pose
        file;       % Object's .ply file
        mesh;       % Object's trisurf object
        pointCloud = []; % Initialize as empty

    end

    methods
        function self = BreadObject(position, pose, file)
            self.position = position;
            self.pose = pose; % For rotation
            self.file = file;
            self.PlotObject();
            self.pointCloud = []; % Initialize as empty

        end

        function PlotObject(self)
            % Load the object model and plot it at the given position
            [faces, vertices, data] = plyread(self.file, 'tri');
            if isfield(data.vertex, 'red') && isfield(data.vertex, 'green') && isfield(data.vertex, 'blue')
                vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            else
                % Default to grey color if color fields are missing
                vertexColors = repmat([0.5, 0.5, 0.5], size(vertices, 1), 1);
            end

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

        function updatePosition(self, newPose)
            self.pose = newPose;
            self.position = newPose;
            % Update the bread's position based on the robot's end-effector
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            vertexCount = size(vertices, 1);

            % Compute the new position for the brick
            breadPose = newPose * transl(0, 0, 0.15) * troty(-pi/2) * trotz(pi);  % Adjust Z-axis offset for gripping
            updatedPoints = [breadPose * [vertices, ones(vertexCount, 1)]']';

            % Update the vertices of the brick's mesh to reflect its new position
            self.mesh.Vertices = updatedPoints(:, 1:3);
        end

        function releaseObject(self)
            % Optional: Remove the mesh if the object is released permanently
            delete(self.mesh);
            self.mesh = [];
            disp('Bread object has been removed from the environment.');
        end


function changeFile(self, newFile)
    % Update the PLY file associated with the object
    self.file = newFile;

    % Load the vertices and colors from the new PLY file
    [faces, vertices, data] = plyread(self.file, 'tri');
    if isfield(data.vertex, 'red') && isfield(data.vertex, 'green') && isfield(data.vertex, 'blue')
        vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    else
        % Default to grey color if color fields are missing
        vertexColors = repmat([0.5, 0.5, 0.5], size(vertices, 1), 1);
    end

    % Apply a corrective rotation around the y-axis
    correctionRotation = troty(pi/2);  % Rotate by -π/2 around the y-axis
    rotatedVertices = (correctionRotation(1:3, 1:3) * vertices')';

    % Transform rotated vertices using the existing pose transformation
    vertexCount = size(rotatedVertices, 1);
    transformedVertices = [self.pose * [rotatedVertices, ones(vertexCount, 1)]']';
    transformedVertices = transformedVertices(:, 1:3); % Discard the homogeneous coordinate

    % Update mesh vertices and faces with the new file's shape and colors
    set(self.mesh, 'Faces', faces, ...
        'Vertices', transformedVertices, ...
        'FaceVertexCData', vertexColors);
    drawnow;
end




        function updateStatus(self, status)
            % Update the object's status and change its PLY file accordingly
            switch status
                case 'bread'
                    self.changeFile('bread.ply');
                    disp('Bread is bread.');
                case 'toasted'
                    self.changeFile('toast.ply');
                    self.pointCloud = self.loadToastPointCloud(self.pose, 'toast.ply');
                    disp('Bread has been toasted.');
                case 'buttered'
                    self.changeFile('toastButtered.ply');
                    disp('Toast has been buttered.');
                case 'released'
                    disp('Object has been released.');
                case 'knife'
                    self.changeFile('knife.ply');
                    disp('object is a knife.');
                otherwise
                    disp(['Unknown status: ', status]);
            end
        end

            function pointCloudGlobal = loadToastPointCloud(self, pose, ply)
            % Load the toast point cloud with transformation
            pointCloudLocal = pcread(ply);
            pointsLocal = pointCloudLocal.Location;
            pointCloudGlobal = (pose * [pointsLocal, ones(size(pointsLocal, 1), 1)]')';
            pointCloudGlobal = pointCloudGlobal(:, 1:3);
        end
    end
end
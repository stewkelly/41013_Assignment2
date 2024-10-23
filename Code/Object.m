classdef Object < handle
    properties
        position;   % Object's [x,y,z] position
        file;       % Object's .ply file
        mesh;       % Object's trisurf object
    end

    methods
        function self = Object(position, file)
            self.position = position;
            self.file = file;
            self.PlotObject();
        end

        function PlotObjectself)
            % Load the object model and plot it at the given position
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Plot the object at the initial position and store the mesh handle
            self.mesh = trisurf(faces, vertices(:, 1) + self.position(1), ...
                vertices(:, 2) + self.position(2), ...
                vertices(:, 3) + self.position(3), ...
                'FaceVertexCData', vertexColors, ...
                'EdgeColor', 'none', 'EdgeLighting', 'none');
        end

        function UpdatePosition(self, newPose)
            % Update the object's position based on the robot's end-effector
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            objectVertexCount = size(vertices, 1);

            % Compute the new position for the object
            objectPose = newPose * transl(0, 0, 0.11);  % Adjust Z-axis offset for gripping
            updatedPoints = [objectPose * [vertices, ones(objectVertexCount, 1)]']';

            % Update the vertices of the object's mesh to reflect its new position
            self.mesh.Vertices = updatedPoints(:, 1:3);
        end
    end
end

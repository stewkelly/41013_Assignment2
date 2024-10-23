classdef BreadObject < handle
    properties
        position;   % Object's [x,y,z] position
        file;       % Object's .ply file
        mesh;       % Object's trisurf object
    end

    methods
        function self = BreadObject(position, file)
            self.position = position;
            self.file = file;
            self.PlotObject();
        end

        function PlotObject(self)
            % Load the object model and plot it at the given position
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Plot the object at the initial position and store the mesh handle
            self.mesh = trisurf(faces, ...
                vertices(:, 1) + self.position(1), ...
                vertices(:, 2) + self.position(2), ...
                vertices(:, 3) + self.position(3), ...
                'FaceVertexCData', vertexColors, ...
                'EdgeColor', 'none', ...
                'EdgeLighting', 'none');
        end

        function updatePosition(self, newPosition)
            % Update the object's position and move its mesh accordingly
            self.position = newPosition;

            % Reload the current PLY file at the new position
            [faces, vertices, data] = plyread(self.file, 'tri');
            vertexColors = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Shift vertices based on the new position
            shiftedVertices = vertices + self.position;

            % Update mesh vertices
            try
                set(self.mesh, 'Faces', faces, ...
                    'Vertices', shiftedVertices, ...
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

            % Shift vertices based on position
            shiftedVertices = vertices + self.position;

            % Ensure shiftedVertices is [N x 3]
            if size(shiftedVertices, 2) ~= 3
                error('Shifted vertices must have three columns (x, y, z).');
            end

            % Update mesh vertices and faces
            set(self.mesh, 'Faces', faces, ...
                'Vertices', shiftedVertices, ...
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
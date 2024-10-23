classdef Simulation < handle
    properties
        r1;
        r1Finger1;
        r1Finger2;

        r2;
        r2Finger1;
        r2Finger2;
    end

    methods
        function self = Simulation()
            clf;
            close all;
            self.plotEnvironment();
            self.plotRobots();
        end

        function plotEnvironment(self)
            clf;
            hold on;
            axis tight
            PlaceObject('bread.ply',[0,1.5,1.05]);
            PlaceObject('bread.ply',[0,1.5,1.1]);
            PlaceObject('bread.ply',[0,1.5,1.15]);
            PlaceObject('bread.ply',[0,1.5,1.2]);

            PlaceObject('toast.ply',[0,0.25,1.05]);
            PlaceObject('table.ply',[0,0,0]);
            PlaceObject('table.ply',[0,1.5,0]);
            PlaceObject('table.ply',[2,0,0]);
            PlaceObject('person.ply',[0,-1,0]);
            PlaceObject('person.ply',[2,-1,0]);
            PlaceObject('emergencyStopButton.ply',[2.5,0,1]);
            PlaceObject('emergencyStopButton.ply',[1.5,0,1]);
            PlaceObject('fenceAssembly.ply',[0.05,2.1,-0.5]);
            PlaceObject('plate.ply',[0,0.25,1.1]);
            PlaceObject('plate.ply',[0,1.5,1.1]);
            PlaceObject('plate.ply',[0.5,1,1.1]);
            PlaceObject('toaster.ply',[-0.5,1,1]);

            % Floor
            surf([-1.5,-1.5;3.1,3.1] ...
                ,[-2,5;-2,5] ...
                ,[0.01,0.01;0.01,0.01] ...
                ,'CData',imread('tile.jpg') ...
                ,'FaceColor','texturemap');

            % Left Wall
            surf([-1.5,-1.5;-1.5,-1.5] ...
                ,[-2,5;-2,5] ...
                ,[0,0;2,2] ...
                ,'CData',imread('tile.jpg') ...
                ,'FaceColor','texturemap');

            % Back Wall
            surf([-1.5,3.1;-1.5,3.1] ...
                ,[5,5;5,5] ...
                ,[0,0;2,2] ...
                ,'CData',imread('tile.jpg') ...
                ,'FaceColor','texturemap');
        end

        function plotRobots(self)
            % XArm6 and Gripper Plotting
            baseR1 = transl(0,1,1);
            self.r1 = XArm6(baseR1);
            q1 = zeros(1,6)
            base = self.r1.model.fkine(q1).T;
            self.r1Finger1 = GripperFinger(base * trotx(pi/2));
            self.r1Finger2 = GripperFinger(base * trotx(pi/2) * troty(pi));

            % Dobot Magician Plotting
            baseR2 = transl(0.5,1.5,1)
            self.r2 = DobotMagician(baseR2);
            q2 = zeros(1,5);

            % Animate Robots
            self.r1.model.animate(q1);
            self.r2.model.animate(q2);
        end

        function runSim()
            for i = 1:numel(self.breads)
                % Set current bread
                self.currentBread = self.breads{i};

                % Control Loop
                % grab (bread)
                % moveArm (to toaster)
                % release (bread)
                % grab (toast)
                % moveArm (to spread)
                % release (toast)
                % spreadButter
                % grab (spreadToast)
                % moveArm (to plate)
                % release (spreadToast)
            end

        end
    end
end

%% Need To Do


%% Additions
% > GUI
% > E-stop
% > Collision Avoidance
% > Singularity

%% Classes
%% Object Class
% to store position and status of bread / toast
% function updatePosition

%% Robot Class
% to contain the robot and gripper functions below

%% function moveArm(self, robot, position)
% qNow = robot.model.getpos();
% targetPose = transl(position) * transl(0, 0, 0.15) * troty(pi);
% qTarget = robot.model.ikcon(targetPose, qNow);
% qMatrix = jtraj(qNow, qTarget, 50);
% disp(qTarget);
% self.AnimateMovement(robot, qMatrix);
% end

%% function moveGripper(self, pose)
% self.r1Finger1.model.base = pose * trotx(pi/2);
% self.r1Finger2.model.base = pose * trotx(pi/2) * troty(pi);
% end

%% function release()
% qNow = self.r1Finger1.model.getpos();
% qOpen = [deg2rad(10), deg2rad(0)];
% steps = 50;
% qMatrix = jtraj(qNow, qOpen, steps);
% 
% for i = 1:steps
%     self.r1Finger1.model.animate(qMatrix(i,:));
%     self.r2Finger2.model.animate(qMatrix(i,:));
%     pause(0.01);
% end

%% function grab()
% qNow = self.r1Finger1.model.getpos();
% qClose = [deg2rad(20), deg2rad(12)];
% steps = 50;
% qMatrix = jtraj(qNow, qOpen, steps);
% 
% for i = 1:steps
%     self.r1Finger1.model.animate(qMatrix(i,:));
%     self.r2Finger2.model.animate(qMatrix(i,:));
%     pause(0.01);
% end

%% function animateMovement(self, robot, qMatrix)
% for i = 1:size(qMatrix, 1)
    % robot.model.animate(qMatrix(i, :));
    % 
    % endEffectorPose = robot.model.fkine(qMatrix(i, :)).T;
% end



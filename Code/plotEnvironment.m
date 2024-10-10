function plotEnvironment()
    clf;
    hold on;
    axis tight
    obj1 = PlaceObject('bread.ply',[0.5,4,1.05]);
    obj2 = PlaceObject('toast.ply',[0.5,0,1.05]);
    obj3 = PlaceObject('table.ply',[0,0,0]);
    obj4 = PlaceObject('table.ply',[0,2,0]);
    obj5 = PlaceObject('table.ply',[0,4,0]);
    obj6 = PlaceObject('table.ply',[2,0,0]);
    obj7 = PlaceObject('table.ply',[2.1,2,0]);
    obj8 = PlaceObject('person.ply',[0,-1,0]);
    obj8 = PlaceObject('person.ply',[2,-1,0]);
    obj10 = PlaceObject('emergencyStopButton.ply',[2.5,0,1]);
    obj11 = PlaceObject('emergencyStopButton.ply',[1.5,0,1]);
    obj12 = PlaceObject('fenceAssembly.ply',[0.05,2.35,-0.5]);
    obj13 = PlaceObject('fenceAssembly.ply',[2.15,2.35,-0.5]);
    obj14 = PlaceObject('plate.ply',[0.5,0,1.1]);
    obj15 = PlaceObject('plate.ply',[0.5,4,1.1]);
    obj16 = PlaceObject('plate.ply',[1.5,2,1.1]);
    obj17 = PlaceObject('toaster.ply',[-0.25,1.75,1]);
    
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


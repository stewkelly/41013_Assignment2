function plotEnvironment()
    hold on;
    axis tight
    obj1 = PlaceObject('bread.ply',[0.5,0,1]);
    obj2 = PlaceObject('toast.ply',[-0.5,0,1]);
    obj3 = PlaceObject('table.ply',[0,0,0]);

end


function plotEnvironment()
    clf;
    hold on;
    axis tight
    obj1 = PlaceObject('bread.ply',[0.5,0,1]);
    obj2 = PlaceObject('toast.ply',[-0.5,0,1]);
    obj3 = PlaceObject('table.ply',[0,0,0]);
    obj4 = PlaceObject('table.ply',[0,2,0]);
    obj5 = PlaceObject('table.ply',[0,4,0]);
    obj6 = PlaceObject('table.ply',[2,0,0]);

end


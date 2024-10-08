function Test()
    axis equal;
    r = DobotCR10(transl(0,0,1)*troty(-pi/2));
    
    qMatrix = jtraj(r.model.qlim(:,1),r.model.qlim(:,2),100);

    for i = 1:100
        r.model.animate(qMatrix(i,:));
        drawnow;
        pause(0.1);
    end

end
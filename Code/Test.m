function Test()

clf;
axis padded;
hold on;

q1 = zeros(1,6);
r = XArm6();

qMatrix = jtraj(r.model.qlim(:,1),r.model.qlim(:,2),100);

r.model.teach(q1);
uiwait;

end


% for i = 1:100
%     r.model.animate(qMatrix(i,:));
%     drawnow;
%     pause(0.1);
% end



% 空间球体投影到深度图像

num = 30;
[x,y,z]=sphere(num-1);
mesh(x,y,z);
X = reshape(x*100,num*num,1);
Y = reshape(y*100,num*num,1);
Z = reshape(z*100,num*num,1);
% plot3(X,Y,Z,'r*');

Camera.intrinsic = [554.2547 0 320;0 554.2547 240; 0 0 1];
Camera.extrinsic = [0 -1 0 0;0 0 -1 0;1 0 0 1500;0 0 0 1];
coords=[X';Y';Z'];

hold on; grid on;
for i = 1:length(coords)
    pointInDepth = projectToDepthFromWord(Camera,coords(:,i));
    plot3([pointInDepth(1),pointInDepth(1)],[pointInDepth(2),pointInDepth(2)],[0,pointInDepth(3)],'.r');
    
end
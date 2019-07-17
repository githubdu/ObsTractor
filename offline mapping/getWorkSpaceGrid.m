function coords = getWorkSpaceGrid(workSpace,flag)

    xMin = workSpace.xMin;
    xMax = workSpace.xMax;
    xNum = workSpace.xNum;
    yMin = workSpace.yMin;
    yMax = workSpace.yMax;
    yNum = workSpace.yNum;
    zMin = workSpace.zMin;
    zMax = workSpace.zMax;
    zNum = workSpace.zNum;
    
    Xlim = linspace(xMin,xMax,xNum+1);
    X = Xlim + (xMax-xMin)/xNum/2;
    X = X(1:xNum);
    
    Ylim = linspace(yMin,yMax,yNum+1);
    Y = Ylim + (yMax-yMin)/yNum/2;
    Y = Y(1:xNum);
    
    Zlim = linspace(zMin,zMax,zNum+1);
    Z = Zlim + (zMax-zMin)/zNum/2;
    Z = Z(1:zNum);
    
    coords = zeros(3,xNum*yNum*zNum);
    
    for i = 1:xNum
        for j=1:yNum
            for k=1:zNum
                coords(:,(i-1)*xNum*yNum + (j-1)*yNum + k) = [X(i),Y(j),Z(k)];
            end
        end
    end
    
    if ~flag
        return;
    end
    
    for i = 1:xNum+1
        for k = 1:zNum+1
            plot3([Xlim(i),Xlim(i)],[yMin,yMax],[Zlim(k),Zlim(k)],'k');
            hold on;
        end
    end
    
    for i = 1:xNum+1
        for j = 1:yNum+1
            plot3([Xlim(i),Xlim(i)],[Ylim(j),Ylim(j)],[zMin,zMax],'k');
            hold on;
        end
    end
    
    for i = 1:zNum+1
        for j = 1:yNum+1
            plot3([xMin,xMax],[Ylim(j),Ylim(j)],[Zlim(i),Zlim(i)],'k');
            hold on;
        end
    end
    
%     plot3(coords(1,:),coords(2,:),coords(3,:),'y*');
    
    xlabel('X');ylabel('Y');zlabel('Z');
    
    grid on;

end
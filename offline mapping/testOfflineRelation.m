
% �����ռ�դ�������ͼ�����ص�����ӳ��

% ��λ mm, rad

% ����������
Camera.intrinsic = [554.2547 0 320;0 554.2547 240; 0 0 1];
Camera.extrinsic = [0 -1 0 0;0 0 -1 0;1 0 0 1500;0 0 0 1];

% �����ռ仮��դ�����
num = 30;

% �����ռ�
workSpace.xMin = -1000;
workSpace.xMax = 1000;
workSpace.xNum = num;
workSpace.yMin = -1000;
workSpace.yMax = 1000;
workSpace.yNum = num;
workSpace.zMin = -1000;
workSpace.zMax = 1000;
workSpace.zNum = num;

coords = getWorkSpaceGrid(workSpace,1);

OfflineRelation = cell(640,480);
OutWorkSpace = cell(1);

for i = 1:length(coords)
    i/(num*num*num)
    pointInDepth = projectToDepthFromWord(Camera,coords(:,i));
    u = pointInDepth(1);
    v = pointInDepth(2);
    if (1<=u && u<=640 && 1<=v && v<=480)
        OfflineRelation{u,v} = [OfflineRelation{u,v},...
            [pointInDepth;coords(:,i)]];
    else
        OutWorkSpace{1} = [OutWorkSpace{1},[pointInDepth;coords(:,i)]];
    end
end

depth = [];
for i=1:1:640
    i/640
    for j = 1:1:480
        if ~isempty(OfflineRelation{i,j})
            temp = OfflineRelation{i,j};
            depth = [depth,temp];
        end
    end
end

% figure(1); hold on; grid on;
plot3(depth(4,:),depth(5,:),depth(6,:),'r*');
% 
% figure(2); hold on; grid on;
% plot3(depth(1,:),depth(2,:),depth(3,:),'g.');

% ������֤
workPoint = [];
for i=1:640
    i/640
    for j = 1:480
        if ~isempty(OfflineRelation{i,j})
            temp = OfflineRelation{i,j};
            for k = 1:size(temp,2)
                temp2 = projectToWordFromDepth(Camera,temp(1:3,k));
                workPoint = [workPoint,temp2];
            end
        end
    end
end

figure(3);hold on; grid on;
plot3(workPoint(1,:),workPoint(2,:),workPoint(3,:),'b.');

% д���ļ�
f = fopen('OfflineRelation.txt','w+');

for i=1:640
    for j = 1:480
        if ~isempty(OfflineRelation{i,j})
            temp = OfflineRelation{i,j};
            fprintf(f,'%d %d ',temp(1,1),temp(2,1));
            for k = 1:size(temp,2)    
                fprintf(f,'%d %.2f %.2f %.2f ',temp(3,1),temp(4,k),temp(5,k),temp(6,k));
            end
            fprintf(f,'\n');
        end
    end
end

fclose(f);
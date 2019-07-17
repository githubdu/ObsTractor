function pointInDepth = projectToDepthFromCamera(Camera,pointInCamera)

    fx = Camera.intrinsic(1,1);
    fy = Camera.intrinsic(2,2);
    cx = Camera.intrinsic(1,3);
    cy = Camera.intrinsic(2,3);
    
    pointInDepth = zeros(3,1);
    
    pointInDepth(1) = ceil(pointInCamera(1)*fx/pointInCamera(3) + cx);
    pointInDepth(2) = ceil(pointInCamera(2)*fy/pointInCamera(3) + cy);
    pointInDepth(3) = round( pointInCamera(3));

end
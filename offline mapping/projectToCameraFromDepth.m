function pointInCamera = projectToCameraFromDepth(Camera,pointInDepth)
    fx = Camera.intrinsic(1,1);
    fy = Camera.intrinsic(2,2);
    cx = Camera.intrinsic(1,3);
    cy = Camera.intrinsic(2,3);
    
    pointInCamera = zeros(3,1);
    pointInCamera(3) = pointInDepth(3);
    pointInCamera(2) = (pointInDepth(2) - cy)*pointInCamera(3)/fy;
    pointInCamera(1) = (pointInDepth(1) - cx)*pointInCamera(3)/fx;
end
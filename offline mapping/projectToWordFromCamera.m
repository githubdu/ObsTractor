function pointInWord = projectToWordFromCamera(Camera,pointInCamera)
    
    pointInWord = (Camera.extrinsic)\[pointInCamera;1];
    pointInWord = pointInWord(1:3);
    
end
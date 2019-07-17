function pointInDepth = projectToDepthFromWord(Camera,pointInWord)

    temp = projectToCameraFromWord(Camera,pointInWord);
    pointInDepth = projectToDepthFromCamera(Camera,temp);
    
end
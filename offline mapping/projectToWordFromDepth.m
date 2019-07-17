function pointInWord = projectToWordFromDepth(Camera,pointInDepth)
    temp = projectToCameraFromDepth(Camera,pointInDepth);
    pointInWord = projectToWordFromCamera(Camera,temp);
end
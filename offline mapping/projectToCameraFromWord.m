function pointInCamera = projectToCameraFromWord(Camera,pointInWord)

    pointInCamera = Camera.extrinsic*[pointInWord;1];
    pointInCamera = pointInCamera(1:3);

end
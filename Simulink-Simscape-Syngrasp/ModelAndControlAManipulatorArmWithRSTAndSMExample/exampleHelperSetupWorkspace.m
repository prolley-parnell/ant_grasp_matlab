% Function to set up environment

function exampleHelperSetupWorkspace(ax)

    plane = collisionBox(1.5,1.5,0.05);
    plane.Pose = trvec2tform([0.25 0 -0.025]);
    plane.show('Parent', ax);

    leftShelf = collisionBox(0.25,0.1,0.2);
    leftShelf.Pose = trvec2tform([0.3 -.65 0.1]);
    [~, patchObj] = leftShelf.show('Parent', ax);
    patchObj.FaceColor = [0 0 1];

    rightShelf = collisionBox(0.25,0.1,0.2);
    rightShelf.Pose = trvec2tform([0.3 .65 0.1]);
    [~, patchObj] = rightShelf.show('Parent', ax);
    patchObj.FaceColor = [0 0 1];

    centerTable = collisionBox(0.5,0.3,0.05);
    centerTable.Pose = trvec2tform([0.75 0 0.025]);
    [~, patchObj] = centerTable.show('Parent', ax);
    patchObj.FaceColor = [0 1 0];

    %Add Objects to pick up
    leftWidget = collisionCylinder(0.01, 0.07);
    leftWidget.Pose = trvec2tform([0.3 -0.65 0.225]);
    [~, patchObj] = leftWidget.show('Parent', ax);
    patchObj.FaceColor = [1 0 0];

    rightWidget = collisionBox(0.03, 0.02, 0.07);
    rightWidget.Pose = trvec2tform([0.3 0.65 0.225]);
    [~, patchObj] = rightWidget.show('Parent', ax);
    patchObj.FaceColor = [1 0 0];
end
clear, clc, close all
%Initializing the robot
robot = importrobot('arm_config_547.urdf')
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
ax = gca;
load('wayPts.mat')
plane = collisionBox(1.5,1.5,0.05);
plane.Pose = trvec2tform([0.25 0 -0.025]);
show(plane,'Parent', ax);

leftShelf = collisionBox(0.25,0.1,0.2);
leftShelf.Pose = trvec2tform([0.3 -.65 0.1]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.25,0.1,0.2);
rightShelf.Pose = trvec2tform([0.3 .65 0.1]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

leftWidget = collisionCylinder(0.01, 0.07);
leftWidget.Pose = trvec2tform([0.3 -0.65 0.225]);
[~, patchObj] = show(leftWidget,'Parent',ax);
patchObj.FaceColor = [1 0 0];

rightWidget = collisionBox(0.03, 0.02, 0.07);
rightWidget.Pose = trvec2tform([0.3 0.65 0.225]);
[~, patchObj] = show(rightWidget,'Parent',ax);
patchObj.FaceColor = [1 0 0];

centerTable = collisionBox(0.5,0.3,0.05);
centerTable.Pose = trvec2tform([0.75 0 0.025]);
[~, patchObj] = show(centerTable,'Parent',ax);
patchObj.FaceColor = [0 1 0];
% config = homeConfiguration(robot)
% config(1)
% randomConfig = robot.randomConfiguration;
% tform = getTransform(robot,randomConfig,'link6','link0');
% 
% ik = inverseKinematics('RigidBodyTree',robot);
% ikWeights = [1 1 1 1 1 1];
% initialguess = robot.homeConfiguration;
% [configSoln,solnInfo] = ik('link6',tform,ikWeights,initialguess);
%  show(robot,configSoln)
viztree.StoredConfigurations = wayPts ;          % Waypoints 
tpts = [0 2 4 6 8 10];                        % Time Points 
tvec = 0:0.1:10;                              % Time Vector 
[q,qd,qdd,pp] = cubicpolytraj(viztree.StoredConfigurations,tpts,tvec); 
r = rateControl(10);
viztree.ShowMarker = false;  % Hide the marker 
 
 for i = 1:size(q',1)
     viztree.Configuration = q(:,i);
     waitfor(r);
 end     


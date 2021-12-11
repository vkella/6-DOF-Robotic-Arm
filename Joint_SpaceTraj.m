function Joint_SpaceTraj(switchPose)
robot = importrobot('arm_config_547.urdf','DataFormat','row');
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
robot1 = viztree.RigidBodyTree;
ax = gca;
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

currentRobotJConfig = homeConfiguration(robot1);
% 7 joints
numJoints = numel(currentRobotJConfig);
endEffector = 'link6';

timeStep = 0.1; 
toolSpeed = 0.1; 
jointInit = currentRobotJConfig;
taskInit = getTransform(robot1,jointInit,endEffector);


%Joint Space
 distance= norm(tform2trvec(taskInit)-tform2trvec(rightWidget.Pose));

% % set time
 initTime = 0;
 finalTime = (distance/toolSpeed) - initTime;
 trajTimes = initTime:timeStep:finalTime;
 timeInterval = [trajTimes(1); trajTimes(end)];

ik = inverseKinematics('RigidBodyTree',robot1);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,rightWidget.Pose,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);

jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot1,'MotionType','PDControl');
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));
qn = jointFinal;
ctrlpoints = [q0,qn];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);
show(robot1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
% Return to initial configuration
show(robot1,currentRobotJConfig,'PreservePlot',false,'Frames','off');

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    viztree.ShowMarker = false;  
    configN = configNow';
    poseNow = getTransform(robot1,configN,endEffector);
    show(robot1,configN,'PreservePlot',false,'Frames','off');
    view(114,18);
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    drawnow;
end
end



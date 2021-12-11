function Cartesian_SpaceTraj(switchPose)
robot = importrobot('arm_config_547.urdf','DataFormat','row');
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
robot1 = viztree.RigidBodyTree
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

% %Task-Space Trajectory
 distance= norm(tform2trvec(taskInit)-tform2trvec(rightWidget.Pose));
% 
% % set time
 initTime = 0;
 finalTime = (distance/toolSpeed) - initTime;
 trajTimes = initTime:timeStep:finalTime;
 timeInterval = [trajTimes(1); trajTimes(end)];
% % Algorithm one
% % Task-Space
 
 tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot1,'EndEffectorName','link6');
 tsMotionModel.Kp(1:3,1:3) = 0;%
 tsMotionModel.Kd(1:3,1:3) = 0;

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));
pose = rightWidget.Pose;
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,pose,t,state),timeInterval,[q0; qd0]);


show(robot1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

for i=1:length(trajTimes)
    % Current time
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);% Algorithm one
    configN = configNow';
    poseNow = getTransform(robot1,configN,endEffector);
    viztree.ShowMarker = false;  % Hide the marker 
    show(robot1,configN,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20)
    drawnow;
end
 
figure(2);
grid on;
plot(tTask,stateTask(:,1:numJoints));
hold all;
plot(tTask(1:numJoints),stateTask(1:numJoints),'--');
title('Joint Position vs Reference ');
 xlabel('Time (s)')
ylabel('Position (rad)');

end

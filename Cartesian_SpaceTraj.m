%##########################################################################
% Plotting the Cartesian Space Trajectory for the Six DOF Arm
%##########################################################################

function Cartesian_SpaceTraj()
robot = importrobot('arm_config_547.urdf','DataFormat','row');
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
robot1 = viztree.RigidBodyTree
ax = gca;

% Setting up the equipment visualization
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


currentPose = homeConfiguration(robot1);
n = numel(currentPose);
endEffector = 'link6';

tstep = 0.1; 
effectorSpeed = 0.1; 
tasktr = getTransform(robot1,currentPose,endEffector);

%Cartesian-Space Trajectory
 diff= norm(tform2trvec(tasktr)-tform2trvec(rightWidget.Pose));

% set time
 initTime = 0;
 finalTime = (diff/effectorSpeed) - initTime;
 pathTime = initTime:tstep:finalTime;
 timeInterval = [pathTime(1); pathTime(end)];

% Task-Space
 armMotionM = taskSpaceMotionModel('RigidBodyTree',robot1,'EndEffectorName','link6');
 armMotionM.Kp(1:3,1:3) = 0;%
 armMotionM.Kd(1:3,1:3) = 0;

q0 = currentPose; 
qd0 = zeros(size(q0));
pose = rightWidget.Pose;

%Solving the differential equation
[Rtime,Rstate] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(armMotionM,timeInterval,tasktr,pose,t,state),timeInterval,[q0; qd0]);

show(robot1,currentPose,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

for i=1:length(pathTime)
    % Current time
    tNow= pathTime(i);
    % Interpolate simulated joint positions to get configuration at current time
    poseNow = interp1(Rtime,Rstate(:,1:n),tNow);
    poseNow_t = poseNow';
    fpose = getTransform(robot1,poseNow_t,endEffector);
    viztree.ShowMarker = false; 
    show(robot1,poseNow_t,'PreservePlot',false,'Frames','off');
    plot3(fpose(1,4),fpose(2,4),fpose(3,4),'b.','MarkerSize',20)
    title('Cartesian Space Trajectory');
    drawnow;
end
hold off;
figure(2);
grid on;
plot(Rtime,Rstate(:,1:n));
hold all;
plot(Rtime(1:n),Rstate(1:n),'--');
title('Joint Position vs Time ');
xlabel('Time (s)')
ylabel('Position (rad)');

end
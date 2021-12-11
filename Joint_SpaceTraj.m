%##########################################################################
% Plotting the Joint Space Trajectory for the Six DOF Arm
%##########################################################################
function Joint_SpaceTraj(switchPose)
robot = importrobot('arm_config_547.urdf','DataFormat','row');
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
robot1 = viztree.RigidBodyTree;
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
% 6 joints
n = numel(currentPose);
endEffector = 'link6';

tstep = 0.1; 
effectorSpeed = 0.1; 
taskTr = getTransform(robot1,currentPose,endEffector);


%Joint Space
 diff= norm(tform2trvec(taskTr)-tform2trvec(rightWidget.Pose));

% set time
 initTime = 0;
 finalTime = (diff/effectorSpeed) - initTime;
 PathTime = initTime:tstep:finalTime;
 timeInterval = [PathTime(1); PathTime(end)];

ik = inverseKinematics('RigidBodyTree',robot1);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
initialGuess = wrapToPi(currentPose);
jointFPose = ik(endEffector,rightWidget.Pose,weights,initialGuess);
jointFPose = wrapToPi(jointFPose);

armMotionJ = jointSpaceMotionModel('RigidBodyTree',robot1,'MotionType','PDControl');
q0 = currentPose; 
qd0 = zeros(size(q0));
qn = jointFPose;
ctrlpoints = [q0,qn];
jPointArray = cubicpolytraj(ctrlpoints,timeInterval,PathTime);

%Solving the differential equation
[jTime,jState] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(armMotionJ,timeInterval,jPointArray,t,state),timeInterval,[q0; qd0]);
show(robot1,currentPose,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

% Return to initial configuration
show(robot1,currentPose,'PreservePlot',false,'Frames','off');

for i=1:length(PathTime)
    % Current time 
    tNow= PathTime(i);
    % Interpolate simulated joint positions to get configuration at current time
    poseNow = interp1(jTime,jState(:,1:n),tNow);
    viztree.ShowMarker = false;  
    poseNow_t = poseNow';
    fPose = getTransform(robot1,poseNow_t,endEffector);
    show(robot1,poseNow_t,'PreservePlot',false,'Frames','off');
    view(114,18);
    plot3(fPose(1,4),fPose(2,4),fPose(3,4),'r.','MarkerSize',20);
    title('Joint Space Trajectory');
    drawnow;
end
hold off;
figure(2);
grid on;
plot(jTime,jState(:,1:n));
hold all;
plot(jTime(1:n),jState(1:n),'--');
title('Joint Position vs Time ');
xlabel('Time (s)')
ylabel('Position (rad)');

end



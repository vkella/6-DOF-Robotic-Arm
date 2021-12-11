%##########################################################################
% Plotting the Workspace for the Six DOF Arm
%##########################################################################
function workspace()
robot = importrobot('arm_config_547.urdf','DataFormat','row');
 T1 = (360+360)*rand() - 360;
 T2 = (135+135)*rand() - 135;
 T3 = (135+135)*rand() - 135;
 T4 = (135+135)*rand() - 135;
 T5 = (360+360)*rand() - 360;
 T6 = (120+120)*rand() - 360;

%Initial joint positions
Q = [T1 T2 T3 T4 T5 T6]';
viztree = interactiveRigidBodyTree(robot,"MarkerBodyName","link6");
viztree.Configuration = Q;
hold on
viztree.ShowMarker = false;
num_samples = 10000;

%Random joint position samples  
for i = 0:num_samples
    T1 = (360+360)*rand() - 360;
    T2 = (135+135)*rand() - 135;
    T3 = (135+135)*rand() - 135;
    T4 = (135+135)*rand() - 135;
    T5 = (360+360)*rand() - 360;
    T6 = (120+120)*rand() - 360;
    Q = [T1 T2 T3 T4 T5 T6 ];
    T = getTransform(robot,Q,'link6');
    %End effector position
    V = tform2trvec(T);
    plot3(V(1),V(2),V(3),'.','Color','b','MarkerSize', 10)
end
title('Workspace of Six Dof Arm')
viztree.Configuration = Q;

hold off
end

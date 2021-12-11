L(1)=Link([pi/2 0 0 0]);
L(2)=Link([pi/2 0 0 pi/2]);
L(3)=Link([pi/2 0 10 0]);
L(4)=Link([pi/2 0 10 pi/2]);
L(5)=Link([pi/2 0 0.5 -pi/2]);
L(6)=Link([pi/2 0.5 0 pi/2]);

robot=SerialLink(L,'name','try');
 T1 = (360+360)*rand() - 360;
 T2 = (135+135)*rand() - 135;
 T3 = (135+135)*rand() - 135;
 T4 = (135+135)*rand() - 135;
 T5 = (360+360)*rand() - 360;
 T6 = (120+120)*rand() - 360;
Q = [T1 T2 T3 T4 T5 T6];
hold on
robot.plot(Q)
for i = 0:10000
    T1 = (360+360)*rand() - 360;
    T2 = (135+135)*rand() - 135;
    T3 = (135+135)*rand() - 135;
    T4 = (135+135)*rand() - 135;
    T5 = (360+360)*rand() - 360;
    T6 = (120+120)*rand() - 360;
    Q = [T1 T2 T3 T4 T5 T6];
    T = robot.fkine(Q);
    a = T.t(1);
    b = T.t(2);
    c = T.t(3);
    plot3(a,b,c,'.','MarkerSize', 10)
end
robot.plot(Q)
hold off


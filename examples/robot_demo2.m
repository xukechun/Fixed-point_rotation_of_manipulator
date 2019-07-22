clear;
clc;
%建立机器人模型
%       theta    d        a        alpha     offset
L1=Link([0       0.4      0.025    pi/2      0     ]); %定义连杆的D-H参数
L2=Link([pi/2    0        0.56     0         0     ]);
L3=Link([0       0        0.035    pi/2      0     ]);
L4=Link([0       0.515    0        pi/2      0     ]);
L5=Link([pi      0        0        pi/2      0     ]);
L6=Link([0       0.08     0        0         0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','manman'); %连接连杆，机器人取名manman
T1=transl(0.5,0,0);%根据给定起始点，得到起始点位姿
T2=transl(0,0.5,0);%根据给定终止点，得到终止点位姿
q1=robot.ikine(T1);%根据起始点位姿，得到起始点关节角
q2=robot.ikine(T2);%根据终止点位姿，得到终止点关节角
[q ,qd, qdd]=jtraj(q1,q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
grid on
T=robot.fkine(q);%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%输出末端轨迹
hold on
robot.plot(q);%动画演示

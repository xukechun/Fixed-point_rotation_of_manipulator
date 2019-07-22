clear;
clc;
%����������ģ��
%       theta    d        a        alpha     offset
L1=Link([0       0.4      0.025    pi/2      0     ]); %�������˵�D-H����
L2=Link([pi/2    0        0.56     0         0     ]);
L3=Link([0       0        0.035    pi/2      0     ]);
L4=Link([0       0.515    0        pi/2      0     ]);
L5=Link([pi      0        0        pi/2      0     ]);
L6=Link([0       0.08     0        0         0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','manman'); %�������ˣ�������ȡ��manman
T1=transl(0.5,0,0);%���ݸ�����ʼ�㣬�õ���ʼ��λ��
T2=transl(0,0.5,0);%���ݸ�����ֹ�㣬�õ���ֹ��λ��
q1=robot.ikine(T1);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q2=robot.ikine(T2);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q ,qd, qdd]=jtraj(q1,q2,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
grid on
T=robot.fkine(q);%���ݲ�ֵ���õ�ĩ��ִ����λ��
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%���ĩ�˹켣
hold on
robot.plot(q);%������ʾ

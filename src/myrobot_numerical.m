clear;
clc;
%% ������������
L=0.1;
d=0.1;
t=0.2032;
k=0.1;

%% ����������ģ��
%�������˵�D-H����
%       theta    d         a        alpha        offset
L1=Link([0  ,   0  ,       0    ,       0     ,      0     ], 'modified'); 
L2=Link([0   ,  0      ,   0     ,     -pi/2    ,    0     ], 'modified');
L3=Link([0  ,   -L    ,     t   ,    0      ,     0     ], 'modified');
L4=Link([-pi/2  ,   0      ,  t  ,    0      ,     0     ], 'modified');
L5=Link([-pi/2  ,   0     ,    0     ,      -pi/2    ,    0     ], 'modified');
L6=Link([0  ,   0    ,     0     ,     -pi/2   ,     0     ], 'modified');
L7=Link([0  ,   d    ,     0     ,      0      ,     0     ], 'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','myrobot'); %��������
%robot.display();
%theta=[0 0 0 -pi/2 -pi/2 0 0];
%robot.plot(theta);

%% ��ʼ�ǶȺ�Ŀ��λ��
%��ʼ�ؽڽǶ��趨���������޸�
theta1=0;
theta2=0.2;
theta3=-0.2;
theta4=0.1;
theta5=0.3;
theta6=-0.15;
theta7=0;

%Ŀ��λ�ü����ʼ��̬���������趨�������۽����ɳ�ʼ״̬����õ㣬Ȼ���Ƹõ㶨��ת��
goal=[    0.9999   -1.0017    0.0105    0.3074
   -1.0017   -0.9982    0.0000   -0.1000
   -0.0105   -0.0000   -0.9999   -0.1007
         0         0         0    1.0000];

%����ĩ����̬�ı仯��ʽ
change=0;

angle=zeros(1,7);

%% ��ѭ����ʵ���Ƽ������ת��
while(1)
    delta_t=0.1;
    while(1)
        %����Ŀ��㣬ֻ�ı���̬�����ı�λ��
        goal=goal*trotz(delta_t)*troty(delta_t);
        
        angle_end=angle;
        %����toolbox�Դ����㷨������˶�ѧ
        angle=robot.ikine(goal,'q0',angle);
        
        %�������С��������СΪ2������������Ϊ50
        if(sum(abs(angle-angle_end))<5)
            step=2;
        else
            step=50;
        end
        trail=trail_generator(angle_end,angle,step);
            
        %չʾ�켣
        robot.plot(trail);
        pause(0.0001);

        %���ĩ��λ��
        robot.fkine(angle)
    
    %��̬�仯Ϊһ������Z�ᣬһ������Y�ύ�����
    if(change==0)
        change=1;
        goal=goal*trotz(pi/2);
    else
        change=0;
        goal=goal*troty(pi/2);
    end
    end
end

    



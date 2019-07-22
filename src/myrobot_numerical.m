clear;
clc;
%% 基本参数设置
L=0.1;
d=0.1;
t=0.2032;
k=0.1;

%% 建立机器人模型
%定义连杆的D-H参数
%       theta    d         a        alpha        offset
L1=Link([0  ,   0  ,       0    ,       0     ,      0     ], 'modified'); 
L2=Link([0   ,  0      ,   0     ,     -pi/2    ,    0     ], 'modified');
L3=Link([0  ,   -L    ,     t   ,    0      ,     0     ], 'modified');
L4=Link([-pi/2  ,   0      ,  t  ,    0      ,     0     ], 'modified');
L5=Link([-pi/2  ,   0     ,    0     ,      -pi/2    ,    0     ], 'modified');
L6=Link([0  ,   0    ,     0     ,     -pi/2   ,     0     ], 'modified');
L7=Link([0  ,   d    ,     0     ,      0      ,     0     ], 'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','myrobot'); %连接连杆
%robot.display();
%theta=[0 0 0 -pi/2 -pi/2 0 0];
%robot.plot(theta);

%% 初始角度和目标位置
%初始关节角度设定，可任意修改
theta1=0;
theta2=0.2;
theta3=-0.2;
theta4=0.1;
theta5=0.3;
theta6=-0.15;
theta7=0;

%目标位置及其初始姿态，可任意设定，操作臂将先由初始状态到达该点，然后绕该点定点转动
goal=[    0.9999   -1.0017    0.0105    0.3074
   -1.0017   -0.9982    0.0000   -0.1000
   -0.0105   -0.0000   -0.9999   -0.1007
         0         0         0    1.0000];

%设置末端姿态的变化形式
change=0;

angle=zeros(1,7);

%% 主循环：实现绕尖端连续转动
while(1)
    delta_t=0.1;
    while(1)
        %对于目标点，只改变姿态，不改变位置
        goal=goal*trotz(delta_t)*troty(delta_t);
        
        angle_end=angle;
        %采用toolbox自带的算法求解逆运动学
        angle=robot.ikine(goal,'q0',angle);
        
        %如果相差较小，步数较小为2，否则步数增加为50
        if(sum(abs(angle-angle_end))<5)
            step=2;
        else
            step=50;
        end
        trail=trail_generator(angle_end,angle,step);
            
        %展示轨迹
        robot.plot(trail);
        pause(0.0001);

        %输出末端位置
        robot.fkine(angle)
    
    %姿态变化为一次绕着Z轴，一次绕着Y轴交替进行
    if(change==0)
        change=1;
        goal=goal*trotz(pi/2);
    else
        change=0;
        goal=goal*troty(pi/2);
    end
    end
end

    



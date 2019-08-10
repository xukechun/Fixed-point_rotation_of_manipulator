# Fixed-point_rotation_of_manipulator六自由度机械臂定点旋转#

本项目中的机械臂只是一个实例，采用改进的DH参数，具体的可以自行修改

## angle_generator.m

采用**PIEPER算法**进行逆运动学规划，得到解析解，此方案在机械臂为六自由度且最后三个关节轴相交时适用，代码中有详细注释，以下为关键步骤

1. 基本参数设置

   ```MATLAB
   %% 基本参数
   L=0.1;
   d=0.1;
   t=0.2032;
   
   %各个关节的DH参数
   alpha0=0;
   a0=0;
   d1=0;
   
   alpha1=-pi/2;
   a1=0;
   d2=0;
   
   alpha2=0;
   a2=t;
   d3=-L;
   
   alpha3=0;
   a3=t;
   d4=0;
   
   alpha4=-pi/2;
   a4=0;
   d5=0;
   
   alpha5=-pi/2;
   a5=0;
   d6=0;
   
   alpha6=0;
   a6=0;
   d7=d;
   theta7=0; %构造一个不动的连杆模拟尖端
   ```

   

2. 求解角度

   ```MATLAB
   %% 求解theta3
   
   %中间变量
   u1=((x^2+y^2+z^2)-L^2-2*t^2)/(2*t^2);
   
   %cos(theta3)=u1存在的限制，如果不符合返回对应的角度为-9999
   if(abs(u1)>1)
       angle(3)=-9999;
       return
   end
   
   %求出八个可能的解
   th3(1)=acos(u1);
   th3(2)=acos(u1);
   th3(3)=acos(u1);
   th3(4)=acos(u1);
   th3(5)=-acos(u1);
   th3(6)=-acos(u1);
   th3(7)=-acos(u1);
   th3(8)=-acos(u1);
   
   %初步筛选，舍去复数根
   th3=th3(find(imag(th3)==0));
   
   %% 求theta2
   
   %中间变量
   f3=a3*sin(alpha2)*sin(th3(1))-d4*sin(alpha3)*sin(alpha2)*cos(th3(1))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2);
   k4=f3*cos(alpha1)+d2*cos(alpha1);
   u2=(z-k4)/sin(alpha1)/sqrt((t*sin(th3(1)))^2+(t+t*cos(th3(1)))^2);
   
   %sin(theta2)=u2存在的限制，如果不符合返回对应的角度为-9999
   if(abs(u2)>1)
       angle(2)=-9999;
       return;
   end
   
   %求出八个可能的解
   th2=zeros(1,8);
   for i=1:2
       th2(i)=asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)));
   end
   
   for i=3:4
       if((z-k4)/sin(alpha1)>=0)
           th2(i)=-asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)))+pi;
       else
           th2(i)=-asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)))-pi;
       end
   end
   
   for i=5:6
       th2(i)=asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)));
   end
   
   for i=7:8
       if((z-k4)/sin(alpha1)>=0)
           th2(i)=-asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)))+pi;
       else
           th2(i)=-asin((z-(a3*sin(alpha2)*sin(th3(i))-d4*sin(alpha3)*sin(alpha2)*cos(th3(i))+d4*cos(alpha2)*cos(alpha3)+d3*cos(alpha2))*cos(alpha1)+d2*cos(alpha1))/sin(alpha1)/sqrt((t*sin(th3(i)))^2+(t+t*cos(th3(i)))^2))-atan2(t*sin(th3(i)),t+t*cos(th3(i)))+pi;
       end
   end
   
   %初步筛选，舍去复数根和角度超过310°的根
   %for k=1:8
   %    if th2(k)>5.41
   %        th2(k)=i;
   %    end
   %end
   th2=th2(find(imag(th2)==0));
   
   %% 求解theta1
   %求出八个可能的解
   th1=zeros(1,8);
   for i=1:4
       u3=x/sqrt(L^2+(t*cos(th2(2*i-1))*cos(th3(2*i-1))-t*sin(th2(2*i-1))*sin(th3(2*i-1))+t*cos(th2(2*i-1)))^2);
       th1(2*i-1)=asin(u3)-atan2(t*cos(th2(2*i-1))*cos(th3(2*i-1))-t*sin(th2(2*i-1))*sin(th3(2*i-1))+t*cos(th2(2*i-1)),L);
       if(u3>=0)
           th1(2*i)=pi-asin(u3)-atan2(t*cos(th2(2*i))*cos(th3(2*i))-t*sin(th2(2*i))*sin(th3(2*i))+t*cos(th2(2*i)),L);
       else
           th1(2*i)=-pi-asin(u3)-atan2(t*cos(th2(2*i))*cos(th3(2*i))-t*sin(th2(2*i))*sin(th3(2*i))+t*cos(th2(2*i)),L);
       end
   end
   
   %sin(theta1)=u3存在的限制，如果不符合返回对应的角度为-9999
   if(abs(u3)>1)
       angle(1)=-9999;
       return;
   end
   
   %初步筛选，舍去复数根和角度超过330°的根
   %for k=1:8
   %    if th1(k)>5.76
   %        th1(k)=i;
   %    end
   %end
   th1=th1(find(imag(th1)==0));
   
   %% 对theta1、theta2、theta3进行选择 
   workspace=1000;
   nearest=0;
   
   %选取最短行程解
   for i=1:length(th1)
       if(imag(th1(i))==0 && imag(th2(i))==0 && imag(th3(i))==0 && abs(th1(i)-q1)+abs(th2(i)-q2)+abs(th3(i)-q3)<workspace)
           workspace=abs(th1(i)-q1)+abs(th2(i)-q2)+abs(th3(i)-q3);
           nearest=i;
       end
   end
   
   %转化为[-pi,pi]
   theta1=wrapToPi(th1(nearest));
   theta2=wrapToPi(th2(nearest));
   theta3=wrapToPi(th3(nearest));
   q1=wrapToPi(th1(nearest));
   q2=wrapToPi(th2(nearest));
   q3=wrapToPi(th3(nearest));
   
   %% 欧拉角求解theta4,theta5,theta6
   
   %齐次变换矩阵
   T_01=[cos(theta1) -cos(alpha0)*sin(theta1) sin(alpha0)*sin(theta1) a0*cos(theta1);
       sin(theta1) cos(theta1)*cos(alpha0) -sin(alpha0)*cos(theta1) sin(theta1)*a0;
       0 sin(alpha0) cos(alpha0) d1;
       0 0 0 1];
   
   T_12=[cos(theta2) -cos(alpha1)*sin(theta2) sin(alpha1)*sin(theta2) a1*cos(theta2);
       sin(theta2) cos(theta2)*cos(alpha1) -sin(alpha1)*cos(theta2) sin(theta2)*a1;
       0 sin(alpha1) cos(alpha1) d2;
       0 0 0 1];
   
   T_23=[cos(theta3) -cos(alpha2)*sin(theta3) sin(alpha2)*sin(theta3) a2*cos(theta3);
       sin(theta3) cos(theta3)*cos(alpha2) -sin(alpha2)*cos(theta3) sin(theta3)*a2;
       0 sin(alpha2) cos(alpha2) d3;
       0 0 0 1];
   
   joint_end_1=joint_end(1:3,1:3);
   joint_end_1=inv(T_23(1:3,1:3))*inv(T_12(1:3,1:3))*inv(T_01(1:3,1:3))*joint_end_1;
   
   %由欧拉角Z-Y-Z求取theta4、theta5、theta6
   th5(1)=atan2(sqrt(joint_end_1(1,3)^2+joint_end_1(2,3)^2),-joint_end_1(3,3));
   th5(2)=atan2(-sqrt(joint_end_1(1,3)^2+joint_end_1(2,3)^2),-joint_end_1(3,3));
   th5=th5(find(imag(th5)==0));
   
   th6(1)=atan2(joint_end_1(3,2)/sin(th5(1)),joint_end_1(3,1)/sin(th5(1)));
   th6(2)=atan2(joint_end_1(3,2)/sin(th5(2)),joint_end_1(3,1)/sin(th5(1)));
   th6=th6(find(imag(th6)==0));
   
   th4(1)=atan2(joint_end_1(2,3)/sin(th5(1)),joint_end_1(1,3)/sin(th5(1)));
   th4(2)=atan2(joint_end_1(2,3)/sin(th5(2)),joint_end_1(1,3)/sin(th5(2)));
   th4=th4(find(imag(th4)==0));
   
   ```

   3.选取最短行程解

   ```matlab
   %选取最短行程解
   workspace=1000;
   nearest=0;
   for i=1:2
       if(double(abs(th5(i)-q5))<double(workspace))
           workspace=abs(th5(i)-q5);
           nearest=i;
       end
   end
   ```

## trail_generator.m

采用五次多项式的拟合进行机械臂的轨迹规划

```matlab
%时间间隔取1s，按步数设置等分
t=(0:(step-1))'/(step-1);
t_step=1;

%起始点和目标点的角位置
q_start=q_start(:);
q_goal=q_goal(:);

%起始点和目标点的角速度
q_start_v=zeros(size(q_start));
q_end_v=q_start_v;

%% 五次多项式拟合系数
a=6*(q_goal-q_start)-3*(q_end_v+q_start_v)*t_step;
b=-15*(q_goal-q_start)+(8*q_start_v+7*q_end_v)*t_step;
c=10*(q_goal-q_start)-(6*q_start_v+4*q_end_v)*t_step;
d=q_start_v*t_step;
e=q_start;
```



## myrobot.m

包括机械臂模型的建立和数据预处理，主函数，并通过matlab的robotics toolbox可视化实现

1. 定义连杆的DH参数模型

   ```matlab
   %       theta    d         a        alpha        offset
   L1=Link([0  ,   0  ,       0    ,       0     ,      0     ], 'modified'); 
   L2=Link([0   ,  0      ,   0     ,     -pi/2    ,    0     ], 'modified');
   L3=Link([0  ,   -L    ,     t   ,    0      ,     0     ], 'modified');
   L4=Link([-pi/2  ,   0      ,  t  ,    0      ,     0     ], 'modified');
   L5=Link([-pi/2  ,   0     ,    0     ,      -pi/2    ,    0     ], 'modified');
   L6=Link([0  ,   0    ,     0     ,     -pi/2   ,     0     ], 'modified');
   L7=Link([0  ,   d    ,     0     ,      0      ,     0     ], 'modified');
   ```

   

2. 采用robotics toolbox实现定点转动

   ```matlab
   while(1)
       delta_t=0.1;
       while(1)
           %对于目标点，只改变姿态，不改变位置
           goal=goal*trotz(delta_t)*troty(delta_t);
           
           %如果不出错，则以前一次得到的解为起点继续求解
           if(angle(1)~=-9999)
               angle_end=angle;
           end
           angle=angle_generator(goal,theta1,theta2,theta3,theta4,theta5,theta6,theta7);
           
           %出错，则跳出循环
           if(angle(1)==-9999 || angle(2)==-9999 || angle(3)==-9999)
               break;
           else
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
               
               %关节角度更新
               q1=angle(1);
               q2=angle(2);
               q3=angle(3);
               q4=angle(4);
               q5=angle(5);
               q6=angle(6);
               q7=angle(7);
               
               %输出末端位置
               robot.fkine(angle)
           end
       end
   ```

   

## myrobot_numerical.m

此方案为数值解，较简单，适用范围较广，采用逆运动学函数进行数值解的解算，采用的是robotics toolbox的ikine函数

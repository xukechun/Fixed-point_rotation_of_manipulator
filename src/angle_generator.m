function angle = angle_generator(goal,q1,q2,q3,q4,q5,q6,q7)
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

%% 尖端位姿转化为最后一个关节的位姿
T_67=[cos(theta7) -cos(alpha6)*sin(theta7) sin(alpha6)*sin(theta7) a6*cos(theta7);
    sin(theta7) cos(theta7)*cos(alpha6) -sin(alpha6)*cos(theta7) sin(theta7)*a6;
    0 sin(alpha6) cos(alpha6) d7;
    0 0 0 1];

%最后一个关节的末端位姿 
joint_end=goal/T_67;

%最后一个关节的末端位置
x=joint_end(1,4);
y=joint_end(2,4);
z=joint_end(3,4);

angle=zeros(1,7);

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

%转化为[-pi,pi]
th4=wrapToPi(th4);
th4=vpa(th4);
th5=wrapToPi(th5);
th5=vpa(th5);
th6=double(th6);
th6=wrapToPi(th6);
th6=vpa(th6);

%选取最短行程解
workspace=1000;
nearest=0;
for i=1:2
    if(double(abs(th5(i)-q5))<double(workspace))
        workspace=abs(th5(i)-q5);
        nearest=i;
    end
end

q4=wrapToPi(th4(nearest));
q5=wrapToPi(th5(nearest));
q6=wrapToPi(th6(nearest));

angle=[double(q1) double(q2) double(q3) double(q4) double(q5) double(q6) double(q7)];

end
function trail = trail_generator(q_start,q_goal,step)
%% 基本参数设置

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

%% 五次多项式形式
time=[t.^5,t.^4,t.^3,t.^2,t,ones(size(t))];
factors=[a,b,c,zeros(size(a)),d,e]';

trail=time*factors;
end


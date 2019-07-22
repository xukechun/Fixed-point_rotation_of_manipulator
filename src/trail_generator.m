function trail = trail_generator(q_start,q_goal,step)
%% ������������

%ʱ����ȡ1s�����������õȷ�
t=(0:(step-1))'/(step-1);
t_step=1;

%��ʼ���Ŀ���Ľ�λ��
q_start=q_start(:);
q_goal=q_goal(:);

%��ʼ���Ŀ���Ľ��ٶ�
q_start_v=zeros(size(q_start));
q_end_v=q_start_v;

%% ��ζ���ʽ���ϵ��
a=6*(q_goal-q_start)-3*(q_end_v+q_start_v)*t_step;
b=-15*(q_goal-q_start)+(8*q_start_v+7*q_end_v)*t_step;
c=10*(q_goal-q_start)-(6*q_start_v+4*q_end_v)*t_step;
d=q_start_v*t_step;
e=q_start;

%% ��ζ���ʽ��ʽ
time=[t.^5,t.^4,t.^3,t.^2,t,ones(size(t))];
factors=[a,b,c,zeros(size(a)),d,e]';

trail=time*factors;
end


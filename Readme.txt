#Fixed-point_rotation_of_manipulator六自由度机械臂定点旋转#
文档中的机械臂只是一个实例，采用改进的DH参数，具体的可以自行修改

（1）angle_generator.m
PIEPER算法进行逆运动学规划，得到解析解，此方案在机械臂为六自由度且最后三个关节轴相交时适用
（2）trail_generator.m
采用五次多项式的拟合进行机械臂的轨迹规划
（3）myrobot.m
包括机械臂模型的建立和数据预处理，主函数，并通过matlab的robotics toolbox可视化实现
（4）myrobot_numerical.m
此方案为数值解，较简单，适用范围较广，采用逆运动学函数进行数值解的解算，采用的是robotics toolbox的ikine函数

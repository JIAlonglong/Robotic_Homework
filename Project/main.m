%%
% author:JIAlonglong
% robot: UR5E
% time: 2023/12/4
%% 
% Build_7DOFROBOT_Lnya:机械臂标准DH参数
% DHFK7Dof_Lnya:生成机械臂的3d模型
% zhengyundongxue：正运动学
% IK_6DOF_Rob_Lnya:逆运动学（全解）
% Ln_IK6DOF:求逆运动学的最优解
% MODWorkSpace:求解机械臂可达的工作空间
% Creat_Jacobian:机器人的动力学相关
% get_ctraj:笛卡尔空间规划（matlab）
% get_jtraj:关节空间规划（matlab）
%%
clear;
clc;
close all;
global T06
%%
ik_fk=0;%正逆解测试
show_forward=0;%运动学正解显示
show_inverse=0;%位置逆解显示(排除了奇异点)
show_best_inverse=0;%求逆运动学最优解（默认最后一组解，轨迹规划时有效）
show_work_space=0;%求解机械臂可达的工作空间
matlab_traj_plan=1;%调用matlab的办法进行轨迹规划
my_trajectory_plan=0;%自己写的轨迹规划代码(苦于时间问题，未能实现)
%%
if ik_fk+show_forward+show_inverse+matlab_traj_plan+my_trajectory_plan>1
    error('run one function one time 一次运行一个选项')
end
%%
if ik_fk
    DHFK7Dof_Lnya(0.1546  ,  0.2124  ,  2.5684  , -3.7169 ,   1.3335 ,   0.5405,100,0,1);
    BB=IK_6DOF_Rob_Lnya(T06)
end    
%% normal setting
%DHFK7Dof_Lnya(0.1546  ,  0.2124  ,  2.5684  , -3.7169 ,   1.3335 ,   0.5405,0,show_forward);
%BB=IK_6DOF_Rob_Lnya(T06)   For test
%%
if show_inverse
    BB=IK_6DOF_Rob_Lnya(T06)
end   
%%
if show_best_inverse
    DHFK7Dof_Lnya(0.1546  ,  0.2124  ,  2.5684  , -3.7169 ,   1.3335 ,   0.5405,100,0,1);
    BB=IK_6DOF_Rob_Lnya(T06)
    best=Ln_IK6DOF(BB)
end
%%
if show_work_space
    theta1min = -165;theta1max = 165;
    theta2min = -95 ;theta2max = 70 ;
    theta3min = -85 ;theta3max = 95 ;
    theta4min = -180;theta4max = 180;
    theta5min = -115;theta5max = 115;
    theta6min = -360;theta6max = 360;
    % 产生3万个随机数
    n = 30000;
    x = zeros;y = zeros;z = zeros;
    for i = 1:n

        theta1 = theta1min*(pi/180) + (theta1max-theta1min)*(pi/180)*rand;
        theta2 = theta2min*(pi/180) + (theta2max-theta2min)*(pi/180)*rand;
        theta3 = theta3min*(pi/180) + (theta3max-theta3min)*(pi/180)*rand;
        theta4 = theta4min*(pi/180) + (theta4max-theta4min)*(pi/180)*rand;
        theta5 = theta5min*(pi/180) + (theta5max-theta5min)*(pi/180)*rand;
        theta6 = theta6min*(pi/180) + (theta6max-theta6min)*(pi/180)*rand;
        theta=[theta1,theta2,theta3,theta4,theta5,theta6];
        Tws = zhengyundongxue(theta);
        x(i) = Tws(1,4);
        y(i) = Tws(2,4);
        z(i) = Tws(3,4);
    end
    figure('color',[1 1 1]);
    plot3(x,y,z,'b.','MarkerSize',0.5)
    hold on
    xlabel('x轴(millimeter)','color','k','fontsize',15);
    ylabel('y轴(millimeter)','color','k','fontsize',15);
    zlabel('z轴(millimeter)','color','k','fontsize',15);
    grid on
end

%% version 2023.12.6
if matlab_traj_plan
    clear;
    clc;
    %建立机器人模型
    L1=Link([0       0.4      0.025    pi/2      0     ]); 
    L2=Link([pi/2    0        0.56     0         0     ]);
    L3=Link([0       0        0.035    pi/2      0     ]);
    L4=Link([0       0.515    0        pi/2      0     ]);
    L5=Link([pi      0        0        pi/2      0     ]);
    L6=Link([0       0.08     0        0         0     ]);
    robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','kaw');
    T1=transl(0.5,0,0);%起始点位姿
    T2=transl(0,-0.5,0);%终止点位姿
    q1=robot.ikine(T1);%起始点关节角
    q2=robot.ikine(T2);%终止点关节角

    %关节空间顾规划
    [q ,qd, qdd]=jtraj(q1,q2,50); 
    T=robot.fkine(q);%根据插值，得到末端执行器位姿
    figure(1)
    robot.plot(q);%动画演示

    %笛卡尔空间顾规划
    Tc=ctraj(T1,T2,20);
    for i=1:20
        qc(i,:)=robot.ikine(Tc(:,:,i));
    end
    figure(2)
    robot.plot(qc);
end


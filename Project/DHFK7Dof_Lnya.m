function pic=DHFK7Dof_Lnya(th1,th2,th3,th4,th5,th6,dz7,fcla,show_forward)
% close all

global Link
global T06

%初始化机器人模型并设置关节的圆柱体的半径和长度
Build_7DOFRobot_Lnya;
radius    = 50;
len       = 150;
joint_col = 0;


plot3(0,0,0,'ro'); 

%关节角度的赋值（角度转换为弧度）
 Link(2).th=th1*pi/180;
 Link(3).th=th2*pi/180;
 Link(4).th=th3*pi/180;
 Link(5).th=th4*pi/180;
 Link(6).th=th5*pi/180;
 Link(7).th=th6*pi/180;
 Link(8).dz=dz7+30;

p0=[0,0,0]';


for i=1:8
Matrix_DH_Ln(i);
end


for i=2:8

      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
      plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
     
      if i<=8
          DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
      end 
     % DrawFrame(Link(i).R, [Link(i).p(1),Link(i).p(2),Link(i).p(3)]' );hold on;%绘制机械臂末端坐标系,可以去掉
end

%计算机械臂末端的矩阵
%DrawFrame(Link(8).R, [Link(8).p(1),Link(8).p(2),Link(8).p(3)]' );hold on;    %绘制机械臂末端坐标系
if show_forward == 1
    theta=[th1,th2,th3,th4,th5,th6];
    T06=zhengyundongxue(theta);
    disp(T06)
end

%网格和坐标轴设置
grid on;
% view(134,12);
axis([-1500,1500,-1500,1500,-1500,1500]);
xlabel('x');
ylabel('y');
zlabel('z');
%更新图形
drawnow;
pic=getframe;
if(fcla)
    cla;
end



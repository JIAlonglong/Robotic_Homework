function Matrix_DH_Ln(i) 
% Caculate the D-H Matrix
global Link

%角度转换常量
ToDeg = 180/pi;
ToRad = pi/180;

%DH参数的计算
C=cos(Link(i).th);%关节角度的余弦
S=sin(Link(i).th);%关节角度的正弦
Ca=cos(Link(i).alf);%联轴器角度的余弦
Sa=sin(Link(i).alf);%联轴器角度的正弦
a=Link(i).dx;    %distance between zi and zi-1（相邻关节的距离）
d=Link(i).dz;    %distance between xi and xi-1

Link(i).n=[C,S,0,0]';%表示相邻坐标系方向的z轴方向。指z轴前一个坐标系在当前坐标系的投影方向。
Link(i).o=[-1*S*Ca,C*Ca,Sa,0]';%表示相邻坐标系的x轴方向。指上一个坐标系中的z轴与当前坐标系的z轴交叉乘积，并进行归一化。
Link(i).a=[S*Sa, -1*C*Sa,Ca,0]';%表示连接相邻坐标系的x轴方向。它指向相邻坐标系的x轴在当前坐标系中的投影方向
Link(i).p=[a*C,a*S,d,1]';%表示连接相邻坐标系的原点（坐标系的起始点）。在D-H参数中，通常表示相邻坐标系的原点在当前坐标系中的坐标

Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];%计算关节之间的旋转矩阵
Link(i).A=[Link(i).n,Link(i).o,Link(i).a,Link(i).p];%计算关节之间的变换矩阵


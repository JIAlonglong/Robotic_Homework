function h = DrawCylinder(pos, az, radius,len, col)
%绘制闭合圆柱体的函数
% pos：圆柱体中心的位置
% az:  圆柱体轴向的方向
% radius：圆柱体的半径
% len：圆柱体的长度
% col：圆柱体的颜色
% draw closed cylinder
%
%******** rotation matrix
%计算旋转矩阵 目标轴：az
az0 = [0;0;1];
ax  = cross(az0,az);
ax_n = norm(ax);
if ax_n < eps 
	rot = eye(3);
else
    ax = ax/ax_n;
    ay = cross(az,ax);
    ay = ay/norm(ay);
    rot = [ax ay az];
end

%********** make cylinder
% col = [0 0.5 0];  % cylinder color
%构建圆柱体的顶点坐标
a = 20;    % number of side faces
theta = (0:a)/a * 2*pi;

x = [radius; radius]* cos(theta);
y = [radius; radius] * sin(theta);
z = [len/2; -len/2] * ones(1,a+1);
cc = col*ones(size(x));

%这部分代码使用之前计算得到的旋转矩阵rot，将圆柱体的顶点坐标旋转到正确的方向。
for n=1:size(x,1)
   xyz = [x(n,:);y(n,:);z(n,:)];
   xyz2 = rot * xyz;
   x2(n,:) = xyz2(1,:);
   y2(n,:) = xyz2(2,:);
   z2(n,:) = xyz2(3,:);
end

%************* draw
% side faces
h = surf(x2+pos(1),y2+pos(2),z2+pos(3),cc);

for n=1:2
	patch(x2(n,:)+pos(1),y2(n,:)+pos(2),z2(n,:)+pos(3),cc(n,:));
end	
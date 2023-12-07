function best_joint=Ln_IK6DOF(BB)
%%  约束条件
% 最短路径（这里的link结构体不断更新全局变量）
global Link
current_joint=[Link(2).th,Link(3).th,Link(4).th,Link(5).th,Link(6).th,Link(7).th];
%% 最短路径
change_degs=0;
min_change_degs=inf;
for i = 1:8
    test_joint(1,1:3)=BB(i,1:3);
    for j=1:3
        change_degs=change_degs+abs(BB(i,j)-current_joint(j));
    end
end

for i = 1:8
    test_joint(1,4:6)=BB(i,4:6);
    for j = 4:6
        change_degs=change_degs+abs(BB(i,j)-current_joint(j))*0.5;
    end
end

if change_degs<min_change_degs
    best_joint=test_joint;
    min_change_degs=change_degs
end

end
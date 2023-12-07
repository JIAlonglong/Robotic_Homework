function T = zhengyundongxue(theta)
    %已知关节角求变换矩阵
    a=[0,-425,-392.25,0,0,0];
    d=[89.459,0,0,109.15,94.65,82.3];
    alpha=[pi/2,0,0,pi/2,-pi/2,0];
 
    
    T01=T_para(theta(1),d(1),a(1),alpha(1));
    T12=T_para(theta(2),d(2),a(2),alpha(2));
    T23=T_para(theta(3),d(3),a(3),alpha(3));
    T34=T_para(theta(4),d(4),a(4),alpha(4));
    T45=T_para(theta(5),d(5),a(5),alpha(5));
    T56=T_para(theta(6),d(6),a(6),alpha(6));
    
    T=T01*T12*T23*T34*T45*T56;
    
end
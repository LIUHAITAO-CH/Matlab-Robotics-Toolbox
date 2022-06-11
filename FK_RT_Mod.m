function T = github_FK_RT_Mod(theta1,theta2,theta3,theta4,theta5,theta6)
%   版本：Matlab          ：2017a
%   版本：Robotic_Toolbox ：V9.10
%   @版权：刘海涛

%   github_FK_RT_Mod函数参数名说明
%   FK ：Forward Kinematics
%   RT ：Robotic_Toolbox
%   Mod：modified
%   综上：基于机器人工具箱的利用改进型的D-H建模方法，计算正运动学：关节角------>转转矩阵

%   参数定义
%   a    ：连杆长度
    a(1) = 0;
    a(2) = 0;
    a(3) = 0.420;
    a(4) = 0.375;
    a(5) = 0;
    a(6) = 0;
%   d    ：连杆偏移
    d(1) =  0;
    d(2) =  0.138;
    d(3) = -0.127;
    d(4) =  0.114;
    d(5) =  0.114;
    d(6) =  0.114;
%   alpha：连杆扭转角
    alpha(1) = 0;
    alpha(2) = -pi/2;
    alpha(3) = 0;
    alpha(4) = 0;
    alpha(5) = -pi/2;
    alpha(6) = pi/2;

    L(1) = Link([0  d(1)  a(1)  alpha(1)], 'modified');
    L(2) = Link([0  d(2)  a(2)  alpha(2)], 'modified');
    L(3) = Link([0  d(3)  a(3)  alpha(3)], 'modified');
    L(4) = Link([0  d(4)  a(4)  alpha(4)], 'modified');
    L(5) = Link([0  d(5)  a(5)  alpha(5)], 'modified');
    L(6) = Link([0  d(6)  a(6)  alpha(6)], 'modified');

    Six_Link = SerialLink(L,'name','sixlink');
    
    Six_Link.plot([theta1 theta2 theta3 theta4 theta5 theta6]);
    
    T = Six_Link.fkine([theta1 theta2 theta3 theta4 theta5 theta6]);
    
end


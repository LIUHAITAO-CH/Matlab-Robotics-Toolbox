有空再完善一下。2022.06.11
各种说明，版本，参数，功能，等等
function theta = Inverse_Kinematics15(T,A)

    
    %定义连杆长度，也可以用一个一维数组****************************************************************************
    a2 = 0.42;
    a3 = 0.375;
    %定义连杆偏距**************************************************************************************************
    d2 =  0.138+0.024;
    d3 = -0.127-0.024;
    d4 =  0.114+0.021;
    d5 =  0.114+0.021;
    d6 =  0.090+0.021;

    r11 =  T(1,1);  r12 =  T(1,2);  r13 =  T(1,3);  Px  =  T(1,4);
    r21 =  T(2,1);  r22 =  T(2,2);  r23 =  T(2,3);  Py  =  T(2,4);
    r31 =  T(3,1);  r32 =  T(3,2);  r33 =  T(3,3);  Pz  =  T(3,4);
    %求解关节角1****************************************************************************************************
    m1 = Py-d6*r23;%为了方便计算，定义的m系列变量
    m2 = Px-d6*r13;
    m3 = d2+d3+d4;
    theta1_1 =  atan2( sqrt(m1^2+m2^2-m3^2) , m3) - atan2(m2 , m1);
    theta1_2 =  atan2(-sqrt(m1^2+m2^2-m3^2) , m3) - atan2(m2 , m1);
    %求解关节角5*****************************************************************************************************
    theta5_1 = atan2(  sqrt(1-(r23*cos(theta1_1)-r13*sin(theta1_1))^2)  ,  (   r23*cos(theta1_1)-r13*sin(theta1_1)  )  );
    theta5_2 = atan2( -sqrt(1-(r23*cos(theta1_1)-r13*sin(theta1_1))^2)  ,  (   r23*cos(theta1_1)-r13*sin(theta1_1)  )  );
    theta5_3 = atan2(  sqrt(1-(r23*cos(theta1_2)-r13*sin(theta1_2))^2)  ,  (   r23*cos(theta1_2)-r13*sin(theta1_2)  )  );
    theta5_4 = atan2( -sqrt(1-(r23*cos(theta1_2)-r13*sin(theta1_2))^2)  ,  (   r23*cos(theta1_2)-r13*sin(theta1_2)  )  );    
    %求解关节角6*****************************************************************************************************
    theta6_1 = atan2(  (r22*cos(theta1_1)-r12*sin(theta1_1))/sin(theta5_1) , -(  r21*cos(theta1_1)-r11*sin(theta1_1)  )/sin(theta5_1)  );
    theta6_2 = atan2(  (r22*cos(theta1_1)-r12*sin(theta1_1))/sin(theta5_2) , -(  r21*cos(theta1_1)-r11*sin(theta1_1)  )/sin(theta5_2)  );
    theta6_3 = atan2(  (r22*cos(theta1_2)-r12*sin(theta1_2))/sin(theta5_3) , -(  r21*cos(theta1_2)-r11*sin(theta1_2)  )/sin(theta5_3)  );
    theta6_4 = atan2(  (r22*cos(theta1_2)-r12*sin(theta1_2))/sin(theta5_4) , -(  r21*cos(theta1_2)-r11*sin(theta1_2)  )/sin(theta5_4)  ); 
    %求解关节角2*****************************************************************************************************
    theta2_theta3_theta4_1 = atan2(   -r33 /sin(theta5_1) ,  (r13*cos(theta1_1) + r23*sin(theta1_1))/sin(theta5_1)   );
    theta2_theta3_theta4_2 = atan2(   -r33 /sin(theta5_2) ,  (r13*cos(theta1_1) + r23*sin(theta1_1))/sin(theta5_2)   );
    theta2_theta3_theta4_3 = atan2(   -r33 /sin(theta5_3) ,  (r13*cos(theta1_2) + r23*sin(theta1_2))/sin(theta5_3)   );
    theta2_theta3_theta4_4 = atan2(   -r33 /sin(theta5_4) ,  (r13*cos(theta1_2) + r23*sin(theta1_2))/sin(theta5_4)   );
    m4_1 = Px*cos(theta1_1) + Py*sin(theta1_1) + d5*sin(theta2_theta3_theta4_1) - d6*sin(theta5_1)*cos(theta2_theta3_theta4_1);
    m4_2 = Px*cos(theta1_1) + Py*sin(theta1_1) + d5*sin(theta2_theta3_theta4_2) - d6*sin(theta5_2)*cos(theta2_theta3_theta4_2);
    m4_3 = Px*cos(theta1_2) + Py*sin(theta1_2) + d5*sin(theta2_theta3_theta4_3) - d6*sin(theta5_3)*cos(theta2_theta3_theta4_3);
    m4_4 = Px*cos(theta1_2) + Py*sin(theta1_2) + d5*sin(theta2_theta3_theta4_4) - d6*sin(theta5_4)*cos(theta2_theta3_theta4_4);
   
    m5_1 = Pz + d5*cos(theta2_theta3_theta4_1) + d6*sin(theta5_1)*sin(theta2_theta3_theta4_1);
    m5_2 = Pz + d5*cos(theta2_theta3_theta4_2) + d6*sin(theta5_2)*sin(theta2_theta3_theta4_2);
    m5_3 = Pz + d5*cos(theta2_theta3_theta4_3) + d6*sin(theta5_3)*sin(theta2_theta3_theta4_3);
    m5_4 = Pz + d5*cos(theta2_theta3_theta4_4) + d6*sin(theta5_4)*sin(theta2_theta3_theta4_4);
    
    m6_1 = 2*m5_1*a2;
    m6_2 = 2*m5_2*a2;
    m6_3 = 2*m5_3*a2;
    m6_4 = 2*m5_4*a2;
    
    m7_1 = 2*m4_1*a2;
    m7_2 = 2*m4_2*a2;
    m7_3 = 2*m4_3*a2;
    m7_4 = 2*m4_4*a2;
    
    m8_1 = a3^2 - a2^2 - m4_1^2 - m5_1^2;
    m8_2 = a3^2 - a2^2 - m4_2^2 - m5_2^2;
    m8_3 = a3^2 - a2^2 - m4_3^2 - m5_3^2;
    m8_4 = a3^2 - a2^2 - m4_4^2 - m5_4^2;
    
    theta2_1 = atan2(  m8_1 ,  sqrt(m6_1^2+m7_1^2-m8_1^2)  )   +  atan2(m7_1,m6_1);
    theta2_2 = atan2(  m8_1 , -sqrt(m6_1^2+m7_1^2-m8_1^2)  )   +  atan2(m7_1,m6_1);
    theta2_3 = atan2(  m8_2 ,  sqrt(m6_2^2+m7_2^2-m8_2^2)  )   +  atan2(m7_2,m6_2);
    theta2_4 = atan2(  m8_2 , -sqrt(m6_2^2+m7_2^2-m8_2^2)  )   +  atan2(m7_2,m6_2);
    theta2_5 = atan2(  m8_3 ,  sqrt(m6_3^2+m7_3^2-m8_3^2)  )   +  atan2(m7_3,m6_3);
    theta2_6 = atan2(  m8_3 , -sqrt(m6_3^2+m7_3^2-m8_3^2)  )   +  atan2(m7_3,m6_3);
    theta2_7 = atan2(  m8_4 ,  sqrt(m6_4^2+m7_4^2-m8_4^2)  )   +  atan2(m7_4,m6_4);
    theta2_8 = atan2(  m8_4 , -sqrt(m6_4^2+m7_4^2-m8_4^2)  )   +  atan2(m7_4,m6_4);
   
    
    %求解关节角3*********************************************************************************************************
    theta2_theta3_1 =  atan2( -(Pz+a2*sin(theta2_1)+d5*cos(theta2_theta3_theta4_1)+d6*sin(theta5_1)*sin(theta2_theta3_theta4_1) ) , ( Px*cos(theta1_1)+Py*sin(theta1_1)-a2*cos(theta2_1)+d5*sin(theta2_theta3_theta4_1)-d6*sin(theta5_1)*cos(theta2_theta3_theta4_1)));
    theta2_theta3_2 =  atan2( -(Pz+a2*sin(theta2_2)+d5*cos(theta2_theta3_theta4_1)+d6*sin(theta5_1)*sin(theta2_theta3_theta4_1) ) , ( Px*cos(theta1_1)+Py*sin(theta1_1)-a2*cos(theta2_2)+d5*sin(theta2_theta3_theta4_1)-d6*sin(theta5_1)*cos(theta2_theta3_theta4_1)));
    theta2_theta3_3 =  atan2( -(Pz+a2*sin(theta2_3)+d5*cos(theta2_theta3_theta4_2)+d6*sin(theta5_2)*sin(theta2_theta3_theta4_2) ) , ( Px*cos(theta1_1)+Py*sin(theta1_1)-a2*cos(theta2_3)+d5*sin(theta2_theta3_theta4_2)-d6*sin(theta5_2)*cos(theta2_theta3_theta4_2)));
    theta2_theta3_4 =  atan2( -(Pz+a2*sin(theta2_4)+d5*cos(theta2_theta3_theta4_2)+d6*sin(theta5_2)*sin(theta2_theta3_theta4_2) ) , ( Px*cos(theta1_1)+Py*sin(theta1_1)-a2*cos(theta2_4)+d5*sin(theta2_theta3_theta4_2)-d6*sin(theta5_2)*cos(theta2_theta3_theta4_2)));
    theta2_theta3_5 =  atan2( -(Pz+a2*sin(theta2_5)+d5*cos(theta2_theta3_theta4_3)+d6*sin(theta5_3)*sin(theta2_theta3_theta4_3) ) , ( Px*cos(theta1_2)+Py*sin(theta1_2)-a2*cos(theta2_5)+d5*sin(theta2_theta3_theta4_3)-d6*sin(theta5_3)*cos(theta2_theta3_theta4_3)));
    theta2_theta3_6 =  atan2( -(Pz+a2*sin(theta2_6)+d5*cos(theta2_theta3_theta4_3)+d6*sin(theta5_3)*sin(theta2_theta3_theta4_3) ) , ( Px*cos(theta1_2)+Py*sin(theta1_2)-a2*cos(theta2_6)+d5*sin(theta2_theta3_theta4_3)-d6*sin(theta5_3)*cos(theta2_theta3_theta4_3)));
    theta2_theta3_7 =  atan2( -(Pz+a2*sin(theta2_7)+d5*cos(theta2_theta3_theta4_4)+d6*sin(theta5_4)*sin(theta2_theta3_theta4_4) ) , ( Px*cos(theta1_2)+Py*sin(theta1_2)-a2*cos(theta2_7)+d5*sin(theta2_theta3_theta4_4)-d6*sin(theta5_4)*cos(theta2_theta3_theta4_4)));
    theta2_theta3_8 =  atan2( -(Pz+a2*sin(theta2_8)+d5*cos(theta2_theta3_theta4_4)+d6*sin(theta5_4)*sin(theta2_theta3_theta4_4) ) , ( Px*cos(theta1_2)+Py*sin(theta1_2)-a2*cos(theta2_8)+d5*sin(theta2_theta3_theta4_4)-d6*sin(theta5_4)*cos(theta2_theta3_theta4_4)));
 
    %求解关节角4*********************************************************************************************************
    theta4_1 = (theta2_theta3_theta4_1)-(theta2_theta3_1);
    theta4_2 = (theta2_theta3_theta4_1)-(theta2_theta3_2);
    theta4_3 = (theta2_theta3_theta4_2)-(theta2_theta3_3);
    theta4_4 = (theta2_theta3_theta4_2)-(theta2_theta3_4);
    theta4_5 = (theta2_theta3_theta4_3)-(theta2_theta3_5);
    theta4_6 = (theta2_theta3_theta4_3)-(theta2_theta3_6);
    theta4_7 = (theta2_theta3_theta4_4)-(theta2_theta3_7);
    theta4_8 = (theta2_theta3_theta4_4)-(theta2_theta3_8);
    
    %求解关节角3*********************************************************************************************************
    theta3_1 = (theta2_theta3_1)-theta2_1;
    theta3_2 = (theta2_theta3_2)-theta2_2;
    theta3_3 = (theta2_theta3_3)-theta2_3;
    theta3_4 = (theta2_theta3_4)-theta2_4;
    theta3_5 = (theta2_theta3_5)-theta2_5;
    theta3_6 = (theta2_theta3_6)-theta2_6;
    theta3_7 = (theta2_theta3_7)-theta2_7;
    theta3_8 = (theta2_theta3_8)-theta2_8;
    
    %将6个关节角展示出来，写成8*6矩阵形式**********************************************************************************
    theta     = [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
                 theta1_1 theta2_2 theta3_2 theta4_2 theta5_1 theta6_1;
                 theta1_1 theta2_3 theta3_3 theta4_3 theta5_2 theta6_2;
                 theta1_1 theta2_4 theta3_4 theta4_4 theta5_2 theta6_2;
                 theta1_2 theta2_5 theta3_5 theta4_5 theta5_3 theta6_3;
                 theta1_2 theta2_6 theta3_6 theta4_6 theta5_3 theta6_3;
                 theta1_2 theta2_7 theta3_7 theta4_7 theta5_4 theta6_4;
                 theta1_2 theta2_8 theta3_8 theta4_8 theta5_4 theta6_4];
    %对数据进行筛选，大于pi，或者小于-pi的解用±2*pi去调整*******************************************************************
    for i=1:8
        for j=1:6
            if  theta(i,j) > pi
				theta(i,j)= - 2*pi + theta(i,j);
            elseif theta(i,j) <-pi  
				theta(i,j)= 2*pi + theta(i,j);
            else 
                theta(i,j)=theta(i,j);
            end
        end
    end
    qiuhe = [0 0 0 0 0 0 0 0];
    for i=1:8
        if (   (theta(i,2)>-pi)   &&   (theta(i,2)<0)   &&    (theta(i,3)>-8*pi/9)   &&    (theta(i,3)<8*pi/9) )
         qiuhe(i) = abs(theta(i,1)-A(1)) + abs(theta(i,2)-A(2)) + abs(theta(i,3)-A(3)) + abs(theta(i,4)-A(4)) + abs(theta(i,5)-A(5)) + abs(theta(i,6)-A(6));
        else
         qiuhe(i) = 100;
        end
    end
    
    [zuixiaozhi,k] =  min(qiuhe);
    
    theta= [theta(k,1) theta(k,2) theta(k,3) theta(k,4) theta(k,5) theta(k,6)];
    
end


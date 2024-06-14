
clc;
clear;
format short;

% 初始值
d1=0.00; d2=0;     d3=14.9; d4=43.3; d5=0; d6=0;
a1=00.0; a2=43.2; a3=-2.0; a4=0;     a5=0; a6=0;
alpha1=-90;alpha2=0;alpha3=90;alpha4=-90;alpha5=90;alpha6=0; 
theta1=0; theta2=0; theta3=0; theta4=0; theta5=0; theta6=0; 
%

% DH-table
joint=['1';'2';'3';'4';'5';'6'];
d= [d1;d2;d3;d4;d5;d6];
a=[a1;a2;a3;a4;a5;a6];
alpha=[alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
theta=[theta1;theta2;theta3;theta4;theta5;theta6];
DH_Table=table(joint,d,a,alpha,theta);
disp(DH_Table)
%





%{

d=[0 0 0.149 0.433 0 0];
alpha=[-90 0 90 -90 90 0];
a=[0 0.432 -0.02 0 0 0];
%}

theta=zeros([8,6]);
T=input("please inuput [nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1;]  可參考附件 IK_input.txt 的格式:\n",'s');
T= str2num(T);

%%%%theta1
%1~4 righty
theta(1:4,1)=atan2(T(2,4),T(1,4))-atan2(d(3),-sqrt(T(2,4)^2+T(1,4)^2-d(3)^2));
%5~8 lefty
theta(5:8,1)=atan2(T(2,4),T(1,4))-atan2(d(3),+sqrt(T(2,4)^2+T(1,4)^2-d(3)^2));

%%%%theta1


%%%%theta3
M=(T(3,4)^2+T(2,4)^2+T(1,4)^2-a(2)^2-a(3)^2-d(3)^2-d(4)^2)/(2*a(2));
%1 2 r_a
theta(1:2,3)=atan2(M,-sqrt(a(3)^2+d(4)^2-M^2)) -atan2(a(3),d(4));
%3 4 r_b
theta(3:4,3)=atan2(M,sqrt(a(3)^2+d(4)^2-M^2)) -atan2(a(3),d(4));
%5 6 l_a
theta(5:6,3)=atan2(M,sqrt(a(3)^2+d(4)^2-M^2)) -atan2(a(3),d(4));
%7 8 l_b
theta(7:8,3)=atan2(M,-sqrt(a(3)^2+d(4)^2-M^2)) -atan2(a(3),d(4));
%%%%theta3



%%%%theta2
%%%R
delta_r=(cos(theta(1,1))*T(1,4) +sin(theta(1,1))*T(2,4))^2+T(3,4)^2;

C23_ra=( (cos(theta(1,1))*T(1,4) +sin(theta(1,1))*T(2,4)) *(a(3)+a(2)*cos(theta(1,3)))   +T(3,4)*(d(4)+a(2)*sin(theta(1,3)))) /delta_r;
S23_ra=( (cos(theta(1,1))*T(1,4) +sin(theta(1,1))*T(2,4)) *(d(4)+a(2)*sin(theta(1,3)))   -T(3,4)*(a(3)+a(2)*cos(theta(1,3)))) /delta_r;
theta23_ra=atan2(S23_ra,C23_ra);
theta(1:2,2)=theta23_ra-theta(1,3);

C23_rb=( (cos(theta(1,1))*T(1,4) +sin(theta(1,1))*T(2,4)) *(a(3)+a(2)*cos(theta(3,3)))   +T(3,4)*(d(4)+a(2)*sin(theta(3,3)))) /delta_r;
S23_rb=( (cos(theta(1,1))*T(1,4) +sin(theta(1,1))*T(2,4)) *(d(4)+a(2)*sin(theta(3,3)))   -T(3,4)*(a(3)+a(2)*cos(theta(3,3)))) /delta_r;
theta23_rb=atan2(S23_rb,C23_rb);
theta(3:4,2)=theta23_rb-theta(3,3);
%%%r

%%%L
delta_l=(cos(theta(5,1))*T(1,4) +sin(theta(5,1))*T(2,4))^2+T(3,4)^2;

C23_la=( (cos(theta(5,1))*T(1,4) +sin(theta(5,1))*T(2,4)) *(a(3)+a(2)*cos(theta(5,3)))   +T(3,4)*(d(4)+a(2)*sin(theta(5,3)))) /delta_l;
S23_la=( (cos(theta(5,1))*T(1,4) +sin(theta(5,1))*T(2,4)) *(d(4)+a(2)*sin(theta(5,3)))   -T(3,4)*(a(3)+a(2)*cos(theta(5,3)))) /delta_l;
theta23_la=atan2(S23_la,C23_la);
theta(5:6,2)=theta23_la-theta(5,3);

C23_lb=( (cos(theta(5,1))*T(1,4) +sin(theta(5,1))*T(2,4)) *(a(3)+a(2)*cos(theta(7,3)))   +T(3,4)*(d(4)+a(2)*sin(theta(7,3)))) /delta_l;
S23_lb=( (cos(theta(5,1))*T(1,4) +sin(theta(5,1))*T(2,4)) *(d(4)+a(2)*sin(theta(7,3)))   -T(3,4)*(a(3)+a(2)*cos(theta(7,3)))) /delta_l;
theta23_lb=atan2(S23_lb,C23_lb);
theta(7:8,2)=theta23_lb-theta(7,3);
%%%l
%%%%theta2


check= theta/pi*180;



%%%T456
T3_inv=zeros([4,4,4]);
T456=zeros([4,4,4]);
% 算奇數  偶數另外調整即可
for i =1:2:7
    T3_inv(:,:, ceil(i/2))=T3_inverse(theta(i,1), theta(i,2) ,theta(i,3),a(2),a(3),d(3));
    T456(:,:,ceil(i/2))=T3_inv(:,:,ceil(i/2))*T;
end
%%%T456




%%%%theta4
for i=1:2:7
    theta(i,4)=atan2(T456(2,3,ceil(i/2)),T456(1,3,ceil(i/2)));
    theta(i+1,4)=theta(i,4)-pi;
end
%%%%theta4

T56=zeros([4,4,4]);
for i=1:4
    T56(:,:,i)=A4_inverse(theta(i*2-1,4),d(4))*T456(:,:,i);
end


for i=1:2:7
    theta(i,5)=atan2(T56(1,3,ceil(i/2)),-1*T56(2,3,ceil(i/2)));
    theta(i+1,5)=-1*theta(i,5);

    theta(i,6)=atan2(T56(3,1,ceil(i/2)),T56(3,2,ceil(i/2)));
    theta(i+1,6)=theta(i,6)-pi;
end

theta_deg=round(theta/pi*180,10);


for i=1:8
    fprintf("================== No.%d's ANS =================================\n",i);
    disp("corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6)")
    if(abs(theta_deg(i,1))>160)
        fprintf("theta1 is out of range!! \n");
    end
    if(abs(theta_deg(i,2))>125)
        fprintf("theta2 is out of range!! \n");
    end 
    if(abs(theta_deg(i,3))>135)
        fprintf("theta3 is out of range!! \n");
    end
    if(abs(theta_deg(i,4))>140)
        fprintf("theta4 is out of range!! \n");
    end
    if(abs(theta_deg(i,5))>100)
    fprintf("theta5 is out of range!! \n");
    end
    if(abs(theta_deg(i,6))>260)
    fprintf("theta6 is out of range!! \n");
    end
    
    disp(theta_deg(i,:))
    
end
disp(" ")
disp("============================END=====================================")




function A4_inv=A4_inverse(theta4,d4)
    A4=[cos(theta4) 0  -sin(theta4)    0;
        sin(theta4) 0  cos(theta4)     0 ;
        0           -1     0           d4 ;
        0           0     0            1  ;];
    A4_inv=inv(A4);
end

function T3_inv= T3_inverse( theta1, theta2 ,theta3 , a2 ,a3,d3 )
    A1 = [cos(theta1)     0             -sin(theta1)      0;
          sin(theta1)     0              cos(theta1)      0 ;
          0              -1              0                0 ;
          0               0              0                1  ;];
 
    A2 = [cos(theta2)    -sin(theta2)    0                a2*cos(theta2);
          sin(theta2)     cos(theta2)    0                a2*sin(theta2) ;
          0               0              1                0 ;
          0               0              0                1  ;];
    A3 = [cos(theta3)     0              sin(theta3)      a3*cos(theta3);
          sin(theta3)     0             -cos(theta3)      a3*sin(theta3) ;
          0               1              0                d3 ;
          0               0              0                1  ;];
    T3 = A1*A2*A3;
    T3_inv=inv(T3);
end

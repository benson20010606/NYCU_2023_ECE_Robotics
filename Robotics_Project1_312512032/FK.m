clc;
clear;

% 參數初始值
d1=0.0; d2=0;     d3=14.9; d4=43.3; d5=0; d6=0;
a1=0.0; a2=43.2; a3=-2; a4=0;     a5=0; a6=0;
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

%請輸入如 [20 20 20 20 20 20 ]
theta=input("please enter the joint variable separated by spaces or commas (in degree) 可參考附件 FK_input.txt 的格式:" + ...
    "\nlimit: theta1  (-160~160); theta2  (-125~125); theta3  (-135~135);\n theta4  (-140~140) ;theta5  (-100~100); theta6  (-260~260):\n",'s');
theta=str2num(theta);

%判斷是否超出範圍
if(abs(theta(1))>160)
    fprintf("theta1 is out of range!! \n");
end
if(abs(theta(2))>125)
    fprintf("theta2 is out of range!! \n");
end 
if(abs(theta(3))>135)
    fprintf("theta3 is out of range!! \n");
end
if(abs(theta(4))>140)
    fprintf("theta4 is out of range!! \n");
end

if(abs(theta(5))>100)
    fprintf("theta5 is out of range!! \n");
end

if(abs(theta(6))>260)
    fprintf("theta6 is out of range!! \n");
end
%

%執行
An=zeros([4 4 6]);
T=eye(4);
for i=1:6
    An(:,:,i)=DH(theta(i),d(i),a(i),alpha(i));
    T=T*An(:,:,i);
end
%

%結果
disp(" ")
%disp("input is :")
%disp(theta);
disp("[n o a p ]:")
disp(T);
[X, Y, Z ,Phi ,Theta, Psi]= noap2RPY(T);
fprintf('\n[X  Y  Z   ϕ  θ  ψ ]:\n');
disp([X,Y,Z,Phi,Theta,Psi])
%

%DH
function transf = DH( Theta,D,A,Alpha)
    
    transf=Trans(0,0,D)*Rot('z',Theta)*Trans(A,0,0)*Rot('x',Alpha);


end

%轉動
function Rot_matrix=Rot(axis,angle)
    angle=angle*pi/180;
    if axis=='x'
        Rot_matrix= [ 1       0            0           0;
                      0     cos(angle)   -sin(angle)   0;
                      0     sin(angle)    cos(angle)   0;
                      0       0            0           1];
    elseif axis=='y'
        Rot_matrix= [ cos(angle)   0    sin(angle)    0;
                      0            1      0           0;
                     -sin(angle)   0    cos(angle)    0;
                      0            0      0           1];
    elseif axis=='z'
        Rot_matrix= [ cos(angle)  -sin(angle)    0   0;
                      sin(angle)  cos(angle)     0   0;
                      0            0             1   0;
                      0            0             0   1];
    end
end

%平移
function Trans_matrix=Trans(X,Y,Z)
    Trans_matrix= [ 1   0  0  X;
                    0   1  0  Y;
                    0   0  1  Z;
                    0   0  0  1];
end

%計算RPY==>ZYZ
function [X,Y,Z,Phi ,Theta, Psi]= noap2RPY(T)
    %RPY==>ZYX
    %Phi=atan2(T(2,1),T(1,1))/pi*180;
    %Theta=asin(-1*T(3,1))/pi*180;
    %Psi=atan2(T(3,2),T(3,3))/pi*180;
    
    %RPY==>ZYZ
    X=T(1,4);
    Y=T(2,4);
    Z=T(3,4);

    if (abs(T(3,3))~=1)
        Psi= atan2(T(2,3),T(1,3));
        Theta=atan2(T(1,3)/cos(Psi),T(3,3))/pi*180;

        Psi=Psi/pi*180;
        Phi=atan2(T(3,2),-T(3,1))/pi*180;
    else
        Theta=0;
        Phi=0;
        Psi=atan2(-1*T(1,2),T(1,1))/pi*180;
    end
   
end

%

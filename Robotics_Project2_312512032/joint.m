clc;clear;
A=[-100.4577    70.6108     48.3997                  0      60.9896     29.2746];  %%利用 project1  輸入齊次矩陣 求得逆向運動學的解
B=[  -52.1158     -1.4358     30.2060       58.6166      11.4781      3.2859];   %%利用 project1  輸入齊次矩陣 求得逆向運動學的解
C=[     0.0955    65.7969     14.3196       15.3377     -20.1730     29.5160];  %%利用 project1  輸入齊次矩陣 求得逆向運動學的解
%%%初始化
theta=zeros([6,501]);
v=zeros([6,501]);
a=zeros([6,501]);
t1=0:0.002:0.3;
t2=0.3:0.002:0.7;
t3=0.7:0.002:1.0;
%%%

%%%   第一段直線段  t= 0.0~0.3
for t=t1
    for i=1:6%六軸
        time=int32(500*t+1); %% matlab index 由 1 開始且為整數 因此將 t= 0.0~0.3  映射到 time = 1~151

        theta(i,time)=(B(i)-A(i))*t/0.5+A(i);  %  推導出的角度公式
        v(i,time)=(B(i)-A(i))/0.5;%  推導出的速度公式
        a(i,time)=0;  %  推導出的加速度公式，角度 為1次式 故為0
    end
end


for t=t2    %第二段曲線段  t= 0.3~0.7
    for i=1:6%六軸
        time=int32(500*t+1);  %  t= 0.0~0.3  映射到 time = 151~351
        
        %推導出來的公式 詳情請看 pdf 檔
        h=(t-0.5+0.2)/0.4;
        a0=(B(i)-A(i))*0.3/0.5+A(i);
        a1=(B(i)-a0)/0.5;
        a3=2*(a0-B(i))+0.8*(C(i)-B(i));
        a4=-0.5*a3;
        theta(i,time)=a0+a1*h+a3*h^3+a4*h^4;
        v(i,time)=(a1+3*a3*h^2+4*a4*h^3)/0.4;
        a(i,time)=(6*a3*h+12*a4*h^2)/0.16;
        %%%
    end
end



for t=t3  %第三段直線段  t= 0.7~1.0
    for i=1:6 %六軸
        time=int32(500*t+1); % t= 0.7~1.0  映射到 time = 351~501
        
        theta(i,time)=B(i)+(C(i)-B(i))*(t-0.5)/0.5;%  推導出的角度公式
        v(i,time)=(C(i)-B(i))/0.5;%  推導出的速度公式
        a(i,time)=0; %  推導出的加速度(位置)公式，角度 為1次式 故為0
    end
end

figure(1) %各軸的角度變化曲線
for i =1:6
    subplot(3,2,i);
    txt = ['Joint  ' int2str(i)];
    plot(0:0.002:1,theta(i,:))
    subtitle(txt);
    if i== 3
    ylabel('angle(degree)')
    end
    xlabel('time(s)')
end

figure(2)%各軸的速度變化曲線
for i =1:6
    subplot(3,2,i);
    txt = ['Joint  ' int2str(i)];
    plot(0:0.002:1,v(i,:))
    subtitle(txt);
    if i== 3
    ylabel('angular velocity (degree/s)')
    end
    xlabel('time(s)')
end

figure(3)%各軸的加速度變化曲線
for i =1:6
    subplot(3,2,i);
    txt = ['Joint  ' int2str(i)];
    plot(0:0.002:1,a(i,:))
    subtitle(txt);
    xlabel('time(s)')
    if i== 3
        ylabel('angular acceleration (degree/s^2)')  
    end
end



%將剛才計算出的各軸的角度變化 透過順向運動學轉換成 位置變化與俯仰角
% DH-table
d1=0.0; d2=0;     d3=14.9; d4=43.3; d5=0; d6=0;
a1=0.0; a2=43.2; a3=-2; a4=0;     a5=0; a6=0;
alpha1=-90;alpha2=0;alpha3=90;alpha4=-90;alpha5=90;alpha6=0; 
d= [d1;d2;d3;d4;d5;d6];
a=[a1;a2;a3;a4;a5;a6];
alpha=[alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
% 參數初始化
x=zeros(1,501);
y=zeros(1,501);
z=zeros(1,501);
%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector
xaf=zeros(1,501);
yaf=zeros(1,501);
zaf=zeros(1,501);
%𝑛𝑜𝑟𝑚𝑎l
xnf=zeros(1,501);
ynf=zeros(1,501);
znf=zeros(1,501);
%𝑜𝑟𝑖𝑒𝑛𝑡𝑎𝑡𝑖𝑜n
xof=zeros(1,501);
yof=zeros(1,501);
zof=zeros(1,501);

for i =1:501 %求得各個時間的齊次矩陣 並得知末端執行器的位置及姿態
    An=zeros([4 4 6]);
    T=eye(4);
    for j=1:6
        An(:,:,j)=DH(theta(j,i),d(j),a(j),alpha(j));
        T=T*An(:,:,j);
    end
    % n o a p 的 p (位置) 
    x(i)=T(1,4); 
    y(i)=T(2,4);
    z(i)=T(3,4);
    % n o a p 的𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ  ( 只顯示這組 不然畫面太亂了 )
    xaf(i) = T(1,3);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  X分量
    yaf(i) = T(2,3);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Y分量
    zaf(i) = T(3,3);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Z分量
    % n o a p 的 𝑛𝑜𝑟𝑚𝑎𝑙 
    xnf(i) = T(1,1);%𝑛𝑜𝑟𝑚𝑎𝑙  vector  X分量
    ynf(i) = T(2,1);%𝑛𝑜𝑟𝑚𝑎𝑙  vector  Y分量
    znf(i) = T(3,1);%𝑛𝑜𝑟𝑚𝑎𝑙  vector  Z分量
    % n o a p 的𝑜𝑟𝑖𝑒𝑛𝑡𝑎𝑡𝑖𝑜𝑛
    xof(i) = T(1,2);%𝑜𝑟𝑖𝑒𝑛𝑡𝑎𝑡𝑖𝑜n vector  X分量
    yof(i) = T(2,2);%𝑜𝑟𝑖𝑒𝑛𝑡𝑎𝑡𝑖𝑜n vector  Y分量
    zof(i) = T(3,2);%𝑜𝑟𝑖𝑒𝑛𝑡𝑎𝑡𝑖𝑜n vector  Z分量
end

%畫出手臂末端在空間中的軌跡與姿態
figure(4)
hold on ;
plot3(5,-55,-60,'o');%點A
text(5,-55,-60,'A (5,-55,-60)')
plot3(50,-40,40,'o');%點B
text(50,-40,40,'B (50,-40,40)')
plot3(60,15,-30,'o');%點C
text(60,15,-30,'C (5,-55,-60)')

plot3(x,y,z,'k', 'LineWidth', 1.5);% 軌跡
%quiver3(x,y,z,xnf,ynf,znf,'r', 'LineWidth', 2')
%quiver3(x,y,z,xof,yof,zof,'g', 'LineWidth', 2')
quiver3(x,y,z,xaf,yaf,zaf,'c', 'LineWidth', 1.5) % 末端點方向 𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector
title('3D path of Joint space planning ')
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')
view([-135,15]) % 圖的視角
grid on;



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


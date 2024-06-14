clc;clear;
%透過 rotm2eul 計算 ZYZ 的euler angle
AA=rotm2eul([0.64 0.77 0 ;0.77 -0.64 0 ;0 0 -1 ;],'ZYZ')/pi*180;
BB=rotm2eul([0.87 -0.1 0.48 ;0.29 0.9  -0.34;-0.4 0.43 0.81;],'ZYZ')/pi*180;
CC=rotm2eul([0.41 -0.29 0.87;0.69 0.71 -0.09;-0.6 0.64 0.49;],'ZYZ')/pi*180;
%XYZ
% 初始化位置與 euler angle
A=[5 -55 -60  AA]; 
B=[50 -40 40 BB];
C=[60 15 -30  CC];
%
%初始化參數儲存空間
postion=zeros([501,6]);  % 位置與 euler angle
v=zeros([501,6]);% 位置與 euler angle 的速度
a=zeros([501,6]);% 位置與 euler angle 的加速度
xaf=zeros(1,501);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  X分量
yaf=zeros(1,501);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Y分量
zaf=zeros(1,501);%𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Z分量

% 時間
t1=0:0.002:0.3;
t2=0.3:0.002:0.7;
t3=0.7:0.002:1.0;
txt=['x','y' ,'z'];


for t=t1  %第一段直線段  t= 0.0~0.3
    for i=1:6  % X Y Z phi theta psi 規劃
        time=int32(500*t+1);%% matlab index 由 1 開始且為整數 因此將 t= 0.0~0.3  映射到 time = 1~151
        
        postion(time,i)=(B(i)-A(i))*t/0.5+A(i);%  推導出的位置與角度公式
        v(time,i)=(B(i)-A(i))/0.5;%  推導出的速度公式
        a(time,i)=0;%  推導出的加速度公式，角度與位置 為1次式 故為0
    end
end


for t=t2   %第二段曲線段  t= 0.3~0.7
    for i=1:6 % X Y Z phi theta psi 規劃
        time=int32(500*t+1); %  t= 0.0~0.3  映射到 time = 151~351
        %推導出來的公式 詳情請看 pdf 檔
        h=(t-0.5+0.2)/0.4;
        a0=(B(i)-A(i))*0.3/0.5+A(i);
        a1=(B(i)-a0)/0.5;
        a3=2*(a0-B(i))+0.8*(C(i)-B(i));
        a4=-0.5*a3;
        postion(time,i)=a0+a1*h+a3*h^3+a4*h^4;
        v(time,i)=(a1+3*a3*h^2+4*a4*h^3)/0.4;
        a(time,i)=(6*a3*h+12*a4*h^2)/0.16;
        %%%
    end
end



for t=t3       %第三段直線段  t= 0.7~1.0
    for i=1:6 % X Y Z phi theta psi 規劃
        time=int32(500*t+1); % t= 0.7~1.0  映射到 time = 351~501

        postion(time,i)=B(i)+(C(i)-B(i))*(t-0.5)/0.5;%推導出的角度與位置公式
        v(time,i)=(C(i)-B(i))/0.5;%  推導出的速度公式
        a(time,i)=0; %  推導出的加速度公式，角度與位置 為1次式 故為0
    end

end



phi=postion(:,4);
theta=postion(:,5);
psi=postion(:,6);

for i = 1:501
    eul = [phi(i) theta(i) psi(i)]/180*pi; 
    rotm = eul2rotm(eul,'ZYZ'); %透過 eul2rotm 將 ZYZ 的euler angle 轉換成齊次矩陣
    xaf(i) =  rotm(1,3); %存取𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  X分量
    yaf(i) = rotm(2,3); %存取𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Y分量
    zaf(i) = rotm(3,3); %存取𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector  Z分量
end



figure(1)%圖1 空間中位置變化曲線
for i =1:3
    subplot(3,1,i);
    plot(0:0.002:1,postion(:,i))
    subtitle(txt(i));
    if i== 2
    ylabel('position(cm)')
    end
    xlabel('time(s)')
end

figure(2)%圖2 速度變化曲線
for i =1:3
    subplot(3,1,i);
    plot(0:0.002:1,v(:,i))
    subtitle(txt(i));
    if i== 2
    ylabel('velocity (cm/s)')
    end
    xlabel('time(s)')
end

figure(3)%圖3 加速度變化曲線
for i =1:3
    subplot(3,1,i);
    plot(0:0.002:1,a(:,i))
    subtitle(txt(i));
    xlabel('time(s)')
    if i==2
        ylabel('acceleration (cm/s^2)')  
    end
end





figure(4)%畫出手臂末端在空間中的軌跡與姿態
hold on ;
plot3(postion(:,1),postion(:,2),postion(:,3),'k', 'LineWidth', 1.5);  % 軌跡
quiver3(postion(:,1),postion(:,2),postion(:,3),xaf',yaf',zaf','c', 'LineWidth', 1.5)% 𝑎𝑝𝑝𝑟𝑜𝑎𝑐ℎ vector 方向

plot3(5,-55,-60,'o'); %點A
text(5,-55,-60,'A (5,-55,-60)')
plot3(50,-40,40,'o');%點B
text(50,-40,40,'B (50,-40,40)')
plot3(60,15,-30,'o');%點C
text(60,15,-30,'C (5,-55,-60)')
title('3D path of Cartesian ')
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')
view([-45,15]) %視角
axis([-20 60 -60 20 -80 40]) % 3D圖範圍
grid on;
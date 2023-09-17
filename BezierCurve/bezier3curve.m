clc;
clear;
close all;
%% 三次贝塞尔曲线
P0=[0,0];
P1=[1,1];
P2=[2,1];
P3=[3,0];
P=[P0;
    P1;
    P2;
    P3];
figure(3);
plot(P(:,1),P(:,2),'k');
MakeGif('三次贝塞尔曲线.gif',1);
hold on
scatter(P(:,1),P(:,2),200,'.b');
 
for t=0:0.01:1
    P_t_1=(1-t) * P0 + t * P1;
    P_t_2=(1-t) * P1 + t * P2;
    P_t_3=(1-t) * P2 + t * P3;
 
    P_t_4=(1-t) * P_t_1 + t * P_t_2;
    P_t_5=(1-t) * P_t_2 + t * P_t_3;
    
    P_t_6=(1-t) * P_t_4 + t * P_t_5;
    
    m1=scatter(P_t_1(1),P_t_1(2),300,'g');
    m2=scatter(P_t_2(1),P_t_2(2),300,'g');
    m3=scatter(P_t_3(1),P_t_3(2),300,'g');
    
    m4=scatter(P_t_4(1),P_t_4(2),300,'m');
    m5=scatter(P_t_5(1),P_t_5(2),300,'m');
    
    m6=plot([P_t_1(1),P_t_2(1),P_t_3(1)],[P_t_1(2),P_t_2(2),P_t_3(2)],'g','linewidth',2);
    m7=plot([P_t_4(1),P_t_5(1)],[P_t_4(2),P_t_5(2)],'m','linewidth',2);
    
    scatter(P_t_6(1),P_t_6(2),300,'.r');  
    
%     stringName = "三次贝塞尔曲线：t="+num2str(t);
%     title(stringName);
    MakeGif('三次贝塞尔曲线.gif',t*100+1);
    delete(m1);
    delete(m2);
    delete(m3);
    delete(m4);
    delete(m5);
    delete(m6);
    delete(m7);
end

function [outputArg1,outputArg2] = MakeGif(name,t)
%该函数用于生成动态GIF图
%   参数1：图像名称 参数2：步长（或时间）
    drawnow
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if t == 1
        imwrite(I,map,name,'gif', 'Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,name,'gif','WriteMode','append','DelayTime',0.2);
    end
end


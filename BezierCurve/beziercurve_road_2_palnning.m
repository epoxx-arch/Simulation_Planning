clc;
clear;
close all;
%% 贝塞尔路径规划
%jubobolv
clc;
clear;
close all;
%% 基本信息 路宽 路长
road_width = 3.6;
road_length = 80;
%起点 障碍物点 目标点
P0=[2 2];
Pob=[40 2.2;
     40 5.5
     ];
Pg=[79 5.5];
P=[P0;Pob;Pg];
i=1;
%% 贝塞尔曲线计算　基本思路Pt＝(1-t)*P1+t*P2
for t=0:0.01:1
    % 一次贝塞尔曲线
    p_t_1=(1-t)*P(1,:)+t*P(2,:);
    p_t_2=(1-t)*P(2,:)+t*P(3,:);
    p_t_3=(1-t)*P(3,:)+t*P(4,:);
    % 二次贝塞尔曲线
    pp_t_1=(1-t)*p_t_1+t*p_t_2;
    pp_t_2=(1-t)*p_t_2+t*p_t_3;
    % 三次贝塞尔曲线
    ppp_t(i,:)=(1-t)*pp_t_1+t*pp_t_2;
    i=i+1;
end
%% 作图
figure
% 道路背景填充
fill([0 road_length road_length 0],[0 0 2*road_width 2*road_width],[0.5,0.5,0.5]);
 
hold on
%车道中间线（两车道中间）
plot([0 road_length],[road_width road_width],'--w','linewidth',2);
%起点
plot(P0(:,1),P0(:,2),'*b');
%障碍物点
plot(Pob(:,1),Pob(:,2),'ob');
%目标点
plot(Pg(:,1),Pg(:,2),'pm');
%绘制贝塞尔规划的路径点
plot(ppp_t(:,1),ppp_t(:,2),'.r');
 
axis equal
set(gca,'XLim',[0 road_length])
set(gca,'YLim',[0 2*road_width])





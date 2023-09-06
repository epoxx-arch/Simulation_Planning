%% 仿真时间要求60秒速度限制15
referenceline_type = 1;
% 全局路径 道路参考线
switch referenceline_type
    case 1
        %直线构成
        ReferenceLine_1_X = 0 : 2 : 1000;
        totle_num = length(ReferenceLine_1_X);
        ReferenceLine_1_Y = zeros(1,totle_num);
        RL1_Left_LaneMarker = zeros(1,totle_num) + 2;
        RL1_Right_LaneMarker = zeros(1,totle_num) - 2;
        ReferenceLine_2_Y = zeros(1,totle_num) + 4;
        RL2_Left_LaneMarker = zeros(1,totle_num) + 6;
    case 2
        % 直线加圆弧
        totle_num = 250;
        ReferenceLine_1_X = zeros(1,totle_num);
        ReferenceLine_1_Y = zeros(1,totle_num);
        RL1_Left_LaneMarker = zeros(1,totle_num);
        RL1_Right_LaneMarker = zeros(1,totle_num);
        ReferenceLine_2_Y = zeros(1,totle_num);
        RL2_Left_LaneMarker = zeros(1,totle_num);
        % 第一段直线 长300米
        segment_1_x = 0 : 3 : 297;
        segment_1_num = length(segment_1_x);
        segment_1_y = zeros(1, segment_1_num);
        ReferenceLine_1_X(1,1:segment_1_num) = segment_1_x;
        ReferenceLine_1_Y(1,1:segment_1_num) = segment_1_y;
        RL1_Left_LaneMarker(1,1:segment_1_num) = (segment_1_y + 2);
        RL1_Right_LaneMarker(1,1:segment_1_num) = segment_1_y - 2;
        ReferenceLine_2_Y(1,1:segment_1_num) = segment_1_y + 4;
        RL2_Left_LaneMarker(1,1:segment_1_num) = segment_1_y + 6;
        % 第二段450米半圆 150个点
        segment_2_x = 300 : 3 : 747;
        segment_2_num = length(segment_2_x);
        segment_2_y = zeros(1, segment_2_num);
        for i = 1:segment_2_num
            segment_2_y(1,i) = 450 - sqrt(450*450 - (segment_2_x(i)-300)^2);
        end
        ReferenceLine_1_X(1, segment_1_num+1 : segment_1_num + segment_2_num) = segment_2_x;
        ReferenceLine_1_Y(1, segment_1_num+1 : segment_1_num + segment_2_num) = segment_2_y;
        for i = 1:segment_2_num
            RL1_Left_LaneMarker(1, segment_1_num + i) = 450 - sqrt(448*448 - (segment_2_x(i)-300)^2);
        end

        for i = 1:segment_2_num
            RL1_Right_LaneMarker(1, segment_1_num + i) = 450 - sqrt(452*452 - (segment_2_x(i)-300)^2);
        end

        for i = 1:segment_2_num
            ReferenceLine_2_Y(1, segment_1_num + i) = 450 - sqrt(446*446 - (segment_2_x(i)-300)^2);
        end

        for i = 1:segment_2_num
            RL2_Left_LaneMarker(1, segment_1_num + i) = 450 - sqrt(444*444 - (segment_2_x(i)-300)^2);
        end
    otherwise
        
end


% 显示全局路径
figure(1);
plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
    ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
    ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-');
axis([-10,300, -50, 50])

%%
%% 获得车道线
%车道1
lane_1_reference_line_x =  ReferenceLine_1_X';
lane_1_reference_line_y =  ReferenceLine_1_Y';
[line_1_reference_line_heading , lane_1_reference_line_kappa] = CalcPathHeadingAndKappa(...
    lane_1_reference_line_x, lane_1_reference_line_y);

%车道2
lane_2_reference_line_x =  ReferenceLine_1_X';
lane_2_reference_line_y =  ReferenceLine_2_Y';
[line_2_reference_line_heading , lane_2_reference_line_kappa] = CalcPathHeadingAndKappa(...
    lane_2_reference_line_x, lane_2_reference_line_y);

%% 自车和静态障碍物信息
ego = struct('x',0,'y',0,'vx',0,'vy',0,'ax',0,'ay',0,'length',0,'width',0,'heading',0,'kappa',0);
ego.x = 0;
ego.y = 0;
ego.vx = 5;
ego.vy = 0;
ego.ax = 0;
ego.ay = 0;
ego.kappa = 0/180*pi;
ego.heading = 0/180*pi;
ego.length = 4;
ego.width = 1.6;

static_object = struct('valid',0,'x',0,'y',0,'v',0,'a',0,'length',0,'width',0,'heading',0);
static_object_num = 10;
static_objects_set = repmat(static_object,static_object_num,1);
%static object 1
static_objects_set(1).valid = 1;
static_objects_set(1).x = 30;
static_objects_set(1).y = -1.2;
static_objects_set(1).v = 0;
static_objects_set(1).heading = 0/180*pi;
static_objects_set(1).length = 4;
static_objects_set(1).width = 2;
% %static object 2
% static_objects_set(2).valid = 1;
% static_objects_set(2).x = 150;
% static_objects_set(2).y = 4;
% static_objects_set(2).v = 0;
% static_objects_set(2).heading = 0/180*pi;
% static_objects_set(2).length = 4;
% static_objects_set(2).width = 2;
% % % static object 3
% static_index = 3;
% static_objects_set(static_index).valid = 1;
% static_objects_set(static_index).x = 200;
% static_objects_set(static_index).y = - 1.0;
% static_objects_set(static_index).v = 0;
% static_objects_set(static_index).heading = 0/180*pi;
% static_objects_set(static_index).length = 4;
% static_objects_set(static_index).width = 2;
% % static object 4
% static_index = 4;
% static_objects_set(static_index).valid = 1;
% static_objects_set(static_index).x = 40;
% static_objects_set(static_index).y = 1.5;
% static_objects_set(static_index).v = 0;
% static_objects_set(static_index).heading = 0/180*pi;
% static_objects_set(static_index).length = 4;
% static_objects_set(static_index).width = 2;
%% 动态障碍物 只有允许 0.2秒一个点 0.2 * 40 = 8秒
dynamic_object = struct('id',0, 'time',zeros(40,1),  'x',zeros(40,1),'y',zeros(40,1), 'vx',0, 'vy',0, ...
    'length',4,'width',2,'heading',0);
front_vehicle = dynamic_object;
front_vehicle.id = 1;
front_vehicle.vx = 5;
front_vehicle.x(1) = 30;
front_vehicle.time(1) = 0;
for i = 2:40
    front_vehicle.x(i) = front_vehicle.x(1) + front_vehicle.vx * 0.2*(i - 1);
    front_vehicle.time(i) = 0.2*(i - 1);
end
% 显示动态障碍物
objWidth = front_vehicle.width;
objLength = front_vehicle.length;
objPosX = front_vehicle.x(1);
objPosY = front_vehicle.y(1);
objHeading = front_vehicle.heading;
Boxyx(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 
Boxyx(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);
patch(Boxyx,Boxyy,'black');
hold on;
plot(front_vehicle.x, front_vehicle.y, 'b.');

%显示ego
objWidth = ego.width;
objLength = ego.length;
objPosX = ego.x;
objPosY = ego.y;
objHeading = ego.heading;
Boxyx(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 
Boxyx(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);
patch(Boxyx,Boxyy,'red');
%axis([ego.x-10,ego.x + 80, -15, 25]);%根据自车定义范围坐标范围

for i = 1:length(static_objects_set)
    objWidth = static_objects_set(i).width;
    objLength = static_objects_set(i).length;
    objPosX = static_objects_set(i).x;
    objPosY = static_objects_set(i).y;
    objHeading = static_objects_set(i).heading;
    Boxyx(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    Boxyy(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
    Boxyx(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    Boxyy(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
    Boxyx(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    Boxyy(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 
    Boxyx(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    Boxyy(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);    
    patch(Boxyx,Boxyy,'black');
end

%axis([static_objects_set(1).x-10,static_objects_set(1).x + 80, -15, 25]);%根据自车定义范围坐标范围



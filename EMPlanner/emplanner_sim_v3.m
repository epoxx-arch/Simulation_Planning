%% 此文件用来测试单一 path trajectory 规划，
%这个是产生变道的横向轨迹
% 自车前方有多个静态的障碍物，根据apollo的public rode planner产生凸空间 然后qp平滑轨迹点
% 规划完毕后 利用single_trajectory_display.m查看是否避障
%% 地图初始化
%直线构成 
ReferenceLine_1_X = 0 : 2 : 1000;
totle_num = length(ReferenceLine_1_X);
ReferenceLine_1_Y = zeros(1,totle_num);
RL1_Left_LaneMarker = zeros(1,totle_num) + 2;
RL1_Right_LaneMarker = zeros(1,totle_num) - 2;
ReferenceLine_2_Y = zeros(1,totle_num) + 4;
RL2_Left_LaneMarker = zeros(1,totle_num) + 6;
% 获得车道线
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
%添加自车的初始状态以及静态障碍物
% % 显示全局路径
% figure(1);
% plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
%     ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
%     ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-');
% axis([-10,300, -50, 50])

%% 自车和静态障碍物信息
ego = struct('x',0,'y',0,'vx',0,'vy',0,'ax',0,'ay',0,'length',0,'width',0,'heading',0,'kappa',0);
ego.x = 5;%
ego.y = 0;%
ego.vx = 5;
ego.vy = 0;
ego.ax = 0;
ego.ay = 0;
ego.kappa = 0/180*pi;
ego.heading = 0/180*pi;
ego.length = 4;
ego.width = 1.6;

object = struct('valid',0,'x',0,'y',0,'v',0,'a',0,'length',0,'width',0,'heading',0);
static_object_num = 10;
static_objects_set = repmat(object,static_object_num,1);
%static object 1
static_objects_set(1).valid = 1;
static_objects_set(1).x = 25;
static_objects_set(1).y = 1.5;
static_objects_set(1).v = 0;
static_objects_set(1).heading = 0/180*pi;
static_objects_set(1).length = 4;
static_objects_set(1).width = 2;


%% 计算投影点
%根据自车的位置在全局的referenceline上面找到自车的投影点
[ego_match_point_index,ego_proj_x, ego_proj_y,ego_proj_heading,ego_proj_kappa] = matchpoint(...
    ego.x ,ego.y ,...
    lane_2_reference_line_x,lane_2_reference_line_y,line_2_reference_line_heading, lane_2_reference_line_kappa);
%在全局的referenceline上截取一段作为规划的路径参考线plan_referenceline
[plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading,plan_referenceline_kappa] = get_plan_referenceline(...
ego_match_point_index,lane_2_reference_line_x,lane_2_reference_line_y,...
line_2_reference_line_heading , lane_2_reference_line_kappa);
%自车的位置x y投影到路径参考线plan_referenceline获得匹配点和投影点坐标朝向曲率
[ego_match_point_index,ego_proj_x, ego_proj_y,ego_proj_heading,ego_proj_kappa] = matchpoint(...
    ego.x ,ego.y ,...
    plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading, plan_referenceline_kappa);
%计算plan_referenceline_s上面的累积 s点
plan_referenceline_s  = get_referenceline_s(plan_referenceline_x,plan_referenceline_y,ego_proj_x,ego_proj_y,ego_match_point_index);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 s l
[ego_plan_start_s,ego_plan_start_l] = Cartesian2Frenet_sl(ego.x,ego.y,plan_referenceline_x,plan_referenceline_y,...
    ego_proj_x,ego_proj_y,ego_proj_heading,ego_match_point_index,plan_referenceline_s);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 dl
[ego_plan_start_s_dot,ego_plan_start_l_dot,ego_plan_start_dl] = Cartesian2Frenet_dsl(ego_plan_start_l,ego.vx,ego.vy,...
    ego_proj_heading,ego_proj_kappa);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 ddl
[~,~,ego_plan_start_ddl] = Cartesian2Frenet_ddsl(ego.ax,ego.ay,...
    ego_proj_heading,ego_proj_kappa,ego_plan_start_l,ego_plan_start_s_dot,ego_plan_start_dl);
%规划起点的frenet
plan_start_s = ego_plan_start_s;
plan_start_l = ego_plan_start_l;
plan_start_dl = ego_plan_start_dl;
plan_start_ddl = ego_plan_start_ddl;
%% 静态障碍物的处理
[filter_static_objects] = static_objects_filter(ego, static_objects_set);
% 需要规划处理的障碍物且计算障碍物的四个角点
% consider_static_object_num = 10 最多处理10个障碍物
consider_static_object_num = length(filter_static_objects);
consider_static_object = struct('id',0,'decision',0,'site',0,'x_set',zeros(4,1),'y_set',zeros(4,1),...
    's_set',zeros(4,1),'l_set',zeros(4,1),'min_s',inf,'max_s',-inf,'min_l',inf,'max_l',-inf);
consider_static_objects_set = repmat(consider_static_object,static_object_num,1);%静态障碍物默认10个
for i = 1 : static_object_num
    if filter_static_objects(i).valid == 0 %invalid
        continue;
    end
    consider_static_objects_set(i).id = i;
    objPosX = filter_static_objects(i).x;
    objPosY = filter_static_objects(i).y;
    objHeading = filter_static_objects(i).heading;
    objWidth = filter_static_objects(i).width;
    objLength = filter_static_objects(i).length;
    consider_static_objects_set(i).x_set(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 

    consider_static_objects_set(i).x_set(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);

    consider_static_objects_set(i).x_set(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);

    consider_static_objects_set(i).x_set(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
end

% 计算考虑静态的障碍物在frenet下面的sl
for i = 1 : static_object_num
    if consider_static_objects_set(i).id == 0
        continue;
    end
    %静态障碍物的四个角点投影到 plan_referenceline 上
    for j = 1 : length(consider_static_objects_set(i).x_set)
        static_obj_x = consider_static_objects_set(i).x_set(j);
        static_obj_y = consider_static_objects_set(i).y_set(j);
        [match_point_index, proj_x, proj_y, proj_heading, proj_kappa] = matchpoint(...
        static_obj_x ,static_obj_y ,...
        plan_referenceline_x, plan_referenceline_y, plan_referenceline_heading, plan_referenceline_kappa);
        [static_obj_s,static_obj_l] = Cartesian2Frenet_sl(static_obj_x, static_obj_y,...
            plan_referenceline_x,plan_referenceline_y,...
            proj_x,proj_y,proj_heading,match_point_index,plan_referenceline_s);
        consider_static_objects_set(i).s_set(j) = static_obj_s;
        consider_static_objects_set(i).l_set(j) = static_obj_l;
        if consider_static_objects_set(i).min_s > static_obj_s
            consider_static_objects_set(i).min_s = static_obj_s;
        end
        if consider_static_objects_set(i).max_s < static_obj_s
            consider_static_objects_set(i).max_s = static_obj_s;
        end
        if consider_static_objects_set(i).min_l > static_obj_l
            consider_static_objects_set(i).min_l = static_obj_l;
        end
        if consider_static_objects_set(i).max_l < static_obj_l
            consider_static_objects_set(i).max_l = static_obj_l;
        end
    end
end

% 添加对静态障碍物的决策
for i = 1 : static_object_num
    if consider_static_objects_set(i).id == 0
        continue;
    end
    if consider_static_objects_set(i).min_l < 0 && consider_static_objects_set(i).max_l < 0
        %障碍物在path_reference右边 1 is site = right 
        consider_static_objects_set(i).site = 1;
        if consider_static_objects_set(i).max_l <= -0.5
            consider_static_objects_set(i).decision = 1;% decision = 1 nudge
        else
            consider_static_objects_set(i).decision = 2;% decision = 2 stop
        end
    end
    if consider_static_objects_set(i).min_l > 0 && consider_static_objects_set(i).max_l > 0
        %障碍物在path_reference左边 2 is site = left 
        consider_static_objects_set(i).site = 2;
        if consider_static_objects_set(i).max_l > 0.8
            consider_static_objects_set(i).decision = 1;% decision = 1 nudge
        else
            consider_static_objects_set(i).decision = 2;
        end
    end
    if consider_static_objects_set(i).min_l < 0 && consider_static_objects_set(i).max_l > 0
        %障碍物在path_reference中间 直接stop决策 3 is site = right
        consider_static_objects_set(i).site = 3;
        consider_static_objects_set(i).decision = 2;
    end
end 

%% 路径的动态规划获得凸空间
% lane keep 凸空间，先初始化为-2 到 2；再处理静态障碍物
% 以规划的起点向前80米 ds = 1
sample_s = 1;
dp_path_s = ones(80,1);
for i = 1:80
    dp_path_s(i) = plan_start_s + (i - 2) * sample_s;
end
% 变道的凸空间
l_min = ones(1,80)*(ego_plan_start_l - 0.4);%add offset
l_max = ones(1,80)*2;
%根据静态障碍物缩小l的凸空间
for i = 1 : static_object_num
    if consider_static_objects_set(i).id == 0
        continue;
    end
    if consider_static_objects_set(i).decision == 1 %decision = 1 nudge
        %根据 min_s 和 max_s 找到开始的点index
        start_index = inf;
        end_index = inf;
        %先找 start index 再找 end_index
        for j = 1 : length(dp_path_s) - 1 
            if dp_path_s(j) <= consider_static_objects_set(i).min_s && ...
                dp_path_s(j+1) >= consider_static_objects_set(i).min_s 
                start_index = max(j - 2,1);%向后增加一个点
                for z = 1 : length(dp_path_s) - 1
                    if dp_path_s(z) <= consider_static_objects_set(i).max_s && ...
                        dp_path_s(z+1) >= consider_static_objects_set(i).max_s 
                        end_index = min(z + 3,length(dp_path_s));%向前增加一个点
                    end
                end
            end
        end
        if start_index ~= inf && end_index ~= inf && (start_index <= end_index)
            %判断在左右  1 is site = right 
            if consider_static_objects_set(i).site(1) == 1
                for k = start_index : end_index
                    l_min(k) = max(consider_static_objects_set(i).max_l + 0.2, l_min(k));
                end
            elseif consider_static_objects_set(i).site(1) == 2 % 2 is site = left 
                for k = start_index : end_index
                    l_max(k) = min(consider_static_objects_set(i).min_l - 0.2, l_max(k));
                end
            else
                 %static in middle stop
            end
        end
    end
end

%% 路径二次规划
% 0.5*x'Hx + f'*x = min
% subject to A*x <= b
%            Aeq*x = beq
%            lb <= x <= ub;
% 输入：l_min l_max 点的凸空间
[qp_path_s, qp_path_l, qp_path_dl, qp_path_ddl] = qp_path(...
    ego_plan_start_s, ego_plan_start_l, ego_plan_start_dl, ego_plan_start_ddl,...
    l_max, l_min);
% frenet坐标系转到世界坐标系 获得初始的路径
[trajectory_x_init, trajectory_y_init, trajectory_heading_init, trajectory_kappa_init] = Frenet2Cartesian_Path(...  
 qp_path_s,qp_path_l, qp_path_dl,qp_path_ddl,...
 plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading, plan_referenceline_kappa,plan_referenceline_s);

%% 动态障碍物



%% 显示车道线 自车 障碍物
%显示规划路径参考线plan_referenceline
figure(2);
% map + 规划的轨迹点
plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-',...
trajectory_x_init, trajectory_y_init,'b.');

% ego
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
%patch(Boxyx,Boxyy,'blue'); 
%静态障碍物
for j = 1:length(static_objects_set)
    objWidth = static_objects_set(j).width;
    objLength = static_objects_set(j).length;
    objPosX = static_objects_set(j).x;
    objPosY = static_objects_set(j).y;
    objHeading = static_objects_set(j).heading;
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
axis([ego.x-10,ego.x + 80, -30, 30]);%根据自车定义范围坐标范围


















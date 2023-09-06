%% speed dp
% lane keep 的dp运算，只有前车目标 cut in目标 以及 横穿目标；

%% 规划的path trajectory 
[trajectory_s_end, trajectory_index2s] = path_trajectory_sum_s(trajectory_x_init, trajectory_y_init);
% 动态障碍物的40个点遍历投影到trajectory上面
front_vehicle_boundary = struct('valid',0, 'id',0, 'min_t',inf,'max_t',-inf, 'num',0, ...
    'time',zeros(40,1), 'lower_points',zeros(40,1), 'upper_points',zeros(40,1), 'guide_s',zeros(40,1));
for i = 1 : 40
    dynamic_obs_x = front_vehicle.x(i);
    dynamic_obs_y = front_vehicle.y(i);
    dynamic_obs_vx = front_vehicle.vx;
    dynamic_obs_vy = front_vehicle.vy;
    % 动态障碍物在trajectory上面投影，找到匹配点和投影点
    [match_point_index, proj_x, proj_y, proj_heading, proj_kappa] = matchpoint(...
        dynamic_obs_x, dynamic_obs_y,...
        trajectory_x_init, trajectory_y_init, trajectory_heading_init, trajectory_kappa_init);
    % 计算 s l
    [start_s,start_l] = Cartesian2Frenet_sl(...
        dynamic_obs_x, dynamic_obs_y, trajectory_x_init, trajectory_y_init,...
        proj_x, proj_y, proj_heading, match_point_index, trajectory_index2s);
    if (start_s >= 0 && start_s - front_vehicle.length <= trajectory_s_end) && abs(start_l) <= 1.2 % 影响自车前行
        front_vehicle_boundary.id = front_vehicle.id;
        front_vehicle_boundary.num = front_vehicle_boundary.num + 1;
        front_vehicle_boundary.time(i) = (i - 1)* 0.2;
        front_vehicle_boundary.lower_points(i) = (start_s - front_vehicle.length/2) - 1;
        front_vehicle_boundary.upper_points(i) = (start_s + front_vehicle.length/2) + 1;
        front_vehicle_boundary.guide_s(i) = start_s - dynamic_obs_vx * 1.5;
        if front_vehicle_boundary.min_t > front_vehicle_boundary.time(i)
            front_vehicle_boundary.min_t = front_vehicle_boundary.time(i);
        end
        if front_vehicle_boundary.max_t < front_vehicle_boundary.time(i)
            front_vehicle_boundary.max_t = front_vehicle_boundary.time(i);
        end
    end
end
% 动态障碍物只有在ST图上至少两个点才有效
if front_vehicle_boundary.num >= 2
    front_vehicle_boundary.valid = 1;
end
% 画图 前车的boundary
for i = 1 : front_vehicle_boundary.num
    plot(front_vehicle_boundary.time(i), front_vehicle_boundary.lower_points(i),  'r.',...
            front_vehicle_boundary.time(i), front_vehicle_boundary.upper_points(i),'r.',...
            front_vehicle_boundary.time(i), front_vehicle_boundary.guide_s(i),'g.')
    hold on;
end
axis([0,10, 0, 70]);

%% 规划起点的信息
% 该函数计算速度规划的初始条件
plan_start_heading = ego.heading;
plan_start_vx = ego.vx;
plan_start_vy = ego.vy;
plan_start_ax = ego.ax;
plan_start_ay = ego.ay;
tor = [cos(plan_start_heading);sin(plan_start_heading)];
% 计算向量v在切向的投影
v_t = tor'*[plan_start_vx;plan_start_vy];
a_t = tor'*[plan_start_ax;plan_start_ay];
plan_start_s_dot = v_t;
plan_start_s_dot2 = a_t;
init_node = struct('v',0,'a',0);
init_node.v = plan_start_s_dot;
init_node.a = plan_start_s_dot2;
% ST图的构建，ds = 0.2米 dt = 0.5秒
% 约束条件的定义
ego_max_velocity = 10;% 自车最大的速度 10m/s
ego_max_acc = 4; % 自车最大的加速度 2m/s^2
ego_min_dece = -6;% 自车最大的减速度 -4m/s^2
ego_max_jerk = 8;% 自车最大的positive jerk
ego_min_jerk = -12;% 自车最大的negtive jerk
% 权重系数
jerk_weight = 30;
acc_weight = 30;
vmax_weight = 40;
pos_weight = 100;

ds = 0.2;
dt = 0.5;
row_num = ceil(trajectory_s_end/0.2);
col_num = 16;%8s /0.5
% valid 节点是否有效
% cost 节点的总代价
% father_row 父节点的行
% a v 此节点的加速度a 和 v
dp_st_node = struct('valid',0,'cost',inf,'father_row',0, 'a',0,'v',0);
dp_st_graph = repmat(dp_st_node, row_num, col_num);
debug_dp_st_gragh_cost = ones(row_num, col_num)*inf;
%% 先计算第一列的的cost t = 0.5
t = 0.5;%第一列的时间0.5秒
[s_max, s_min, valid] = CalcSRange(t, plan_start_s_dot, front_vehicle_boundary);
if valid == 1
    next_highest_row = ceil(s_max/0.2);
    next_lowest_row = ceil(s_min/0.2);
else
    pause;
end
%计算cost
for cur_row = next_lowest_row : next_highest_row
    %碰撞检测 线段和障碍物的boundary
    
    %先计算节点转移代价 两点之间是匀jerk运动
    delta_s = cur_row * ds - 0;%节点到初始点的距离差
    %根据 s = v0 * t + 0.5 * a0 * t^2 + 1/6 * jerk * t^3;
    a0 = init_node.a;
    v0 = init_node.v;
    
    vt = 2 * delta_s/dt - v0;
    if vt > ego_max_velocity || vt < 0
        continue;
    end 
    
    acc = 2*(delta_s/dt - v0)/dt;%这一段的平均加速度
    if acc > ego_max_acc || acc < ego_min_dece
        continue;
    end
    at = a0 + acc * dt;
    if at > ego_max_acc || at < ego_min_dece
        continue;
    end
    
    jerk = (at - a0)/dt;
    if jerk > ego_max_jerk || jerk < ego_min_jerk 
        continue;
    end
    
    jerk_cost = abs(jerk/ego_min_jerk) * jerk_weight;
    acc_cost = abs(at/ego_min_dece) * acc_weight;
    vmax_cost = abs(ego_max_velocity - vt)/ego_max_velocity  * vmax_weight;
    % 位置代价 当前节点和前车的位置代价
    pos_cost = 0;
    [valid, s_guide] = CalcSGuide(t, front_vehicle_boundary);
    if valid == 1
        pos_cost = (abs(s_guide - (cur_row -1)*ds)/60) * pos_weight;
    end
    debug_dp_st_gragh_cost(cur_row,1) = (jerk_cost + acc_cost + pos_cost);
    dp_st_graph(cur_row,1).cost = (jerk_cost + acc_cost + pos_cost);
    dp_st_graph(cur_row,1).v = vt;
    dp_st_graph(cur_row,1).a = at;
end

%% 先计算第二列的的cost t = 1
t = 1;%第二列的时间0.5秒
%上一列的范围
last_lowest_row = next_lowest_row;
last__highest_row = next_highest_row;
[s_max, s_min, valid] = CalcSRange(t, plan_start_s_dot, front_vehicle_boundary);
if valid == 1
    next_highest_row = ceil(s_max/0.2);
    next_lowest_row = ceil(s_min/0.2);
else
    pause;
end
%计算cost 值规划12列 6秒
for time_col = 2:12
    for cur_row = next_lowest_row : next_highest_row
        for last_row = last_lowest_row : last__highest_row
            %碰撞检测 线段和障碍物的boundary

            %先计算节点转移代价 两点之间是匀jerk运动
            delta_s = ((cur_row - last_row) - 1) * ds;%节点到初始点的距离差
            %根据 s = v0 * t + 0.5 * a0 * t^2 + 1/6 * jerk * t^3;
            a0 = dp_st_graph(last_row, time_col -1).a;
            v0 = dp_st_graph(last_row, time_col -1).v;
            
            vt = 2 * delta_s/dt - v0;
            if vt > ego_max_velocity || vt < 0
                continue;
            end 
    
            acc = 2*(delta_s/dt - v0)/dt;%这一段的平均加速度
            if acc > ego_max_acc || acc < ego_min_dece
                continue;
            end
            at = a0 + acc * dt;
            if at > ego_max_acc || at < ego_min_dece
                continue;
            end
    
            jerk = (at - a0)/dt;
            if jerk > ego_max_jerk || jerk < ego_min_jerk 
                continue;
            end
            
            jerk_cost = abs(jerk/ego_min_jerk) * jerk_weight;
            acc_cost = abs(at/ego_min_dece) * acc_weight;
            vmax_cost = abs(ego_max_velocity - vt)/ego_max_velocity  * vmax_weight;
            % 位置代价 当前节点和前车的位置代价
            pos_cost = 0;
            [valid, s_guide] = CalcSGuide(t, front_vehicle_boundary);
            if valid == 1
                pos_cost = (abs(s_guide - (cur_row -1)*ds)/60) * pos_weight;
            end
            temp_cost = (jerk_cost + acc_cost + pos_cost);
            if (temp_cost + dp_st_graph(last_row, time_col - 1).cost) < dp_st_graph(cur_row, time_col).cost
                debug_dp_st_gragh_cost(cur_row, time_col) = temp_cost + dp_st_graph(last_row, time_col -1).cost;
                dp_st_graph(cur_row, time_col).cost = temp_cost + dp_st_graph(last_row, time_col -1).cost;
                dp_st_graph(cur_row, time_col).v = vt;
                dp_st_graph(cur_row, time_col).a = at;
                dp_st_graph(cur_row, time_col).father_row = last_row;
            end
        end
    end
    last_lowest_row = next_lowest_row;
    last__highest_row = next_highest_row;
    t = (time_col + 1) * 0.5;
    [s_max, s_min, valid] = CalcSRange(t, plan_start_s_dot, front_vehicle_boundary);
    if valid == 1
        next_highest_row = ceil(s_max/0.2);
        next_lowest_row = ceil(s_min/0.2);
    else
        pause;
    end
end

%先在最后一行和最后一排找到最小值
min_cost = inf;
min_row = 0;
min_col = 0;
father_row = 0;
%在295行的1-12列找
row_num = length(dp_st_graph(:, 1));
for cur_col = 1:12
    if dp_st_graph(row_num, cur_col).cost < min_cost
        min_cost = dp_st_graph(row_num, cur_col).cost;
        min_row = row_num;
        min_col = cur_col;
        father_row = dp_st_graph(row_num, cur_col).father_row;
    end
end
% 12行295列找
for cur_row = 1:row_num
    if dp_st_graph(cur_row, 12).cost < min_cost
        min_cost = dp_st_graph(cur_row, 12).cost;
        min_row = cur_row;
        min_col = cur_col;
        father_row = dp_st_graph(cur_row, 12).father_row;
    end
end

dp_t = 0.5:0.5:6;
dp_s = zeros(1,12);
dp_s(12) = (min_row - 1) * ds;
for i = 11: -1 :1
    dp_s(i) = (father_row - 1) * ds;
    father_row = dp_st_graph(father_row, i).father_row;
end
hold on;
plot(dp_t,dp_s,'*');



% function 输入t 找到前车的 s_guide
% valid 是t是否在前车的min_t 和 max_t内
function [valid, s_guide] = CalcSGuide(t, front_vehicle_boundary)
    valid = 0;
    s_guide = 0;
    if t >= front_vehicle_boundary.min_t && t <= front_vehicle_boundary.max_t
        for i = 1:39
            if t >= front_vehicle_boundary.time(i) && t <= front_vehicle_boundary.time(i+1)
                valid = 1;
                s_guide = (t - front_vehicle_boundary.time(i) )/(front_vehicle_boundary.time(i+1) - front_vehicle_boundary.time(i)) * ...
                    (front_vehicle_boundary.guide_s(i+1) - front_vehicle_boundary.guide_s(i)) + ...
                    front_vehicle_boundary.guide_s(i);
            end
        end
    end

end


%计算在0-8秒的任意时刻可以到达的s的max 和min
% 假设自车的速度小于等于10m/s
function [s_max, s_min, valid] = CalcSRange(t, plan_start_s_dot, front_vehicle_boundary)
% input t 在0-8秒的任意时刻
% plan_start_s_dot 规划起点的速度
s_max = 0;
s_min = 0;
valid = 0;
if t > 8 || t < 0
    return ;
end
% 动力学约束
ego_max_velocity = 10;% 自车最大的速度10m/s ,假设自车的速度小于等于10m/s
ego_max_acc = 2; % 自车最大的加速度2m/s^2
ego_max_dece = -4;%
if plan_start_s_dot > 10
    plan_start_s_dot = 10;
end
% 加速器时间
t0 = (ego_max_velocity - plan_start_s_dot)/ego_max_acc;
% 匀速时间
t1 = 8 - t0;
if t <= t0
    % 匀加速运动 认为规划的计算s0 = 0
    s_max = 0 + plan_start_s_dot * t + 0.5 * ego_max_acc * t^2;
end
if t > t0
    s_max = 0 + plan_start_s_dot * t0 + 0.5 * ego_max_acc * t0^2 + ...
        (t - t0)*ego_max_velocity;
end
% 另外还需要考虑前车 front_vehicle_boundary必须是前车
if front_vehicle_boundary.valid == 1 %前车的boundary合法
    if t >= front_vehicle_boundary.min_t && t <= front_vehicle_boundary.max_t
        for i = 1:39
            if t >= front_vehicle_boundary.time(i) && t <= front_vehicle_boundary.time(i+1)
                s_upper = (t - front_vehicle_boundary.time(i) )/(front_vehicle_boundary.time(i+1) - front_vehicle_boundary.time(i)) * ...
                    (front_vehicle_boundary.lower_points(i+1) - front_vehicle_boundary.lower_points(i)) + ...
                    front_vehicle_boundary.lower_points(i);
                s_max = min(s_max, s_upper);
            end
        end
    end
end

% 减速可以到0
t_dece = (plan_start_s_dot - 0)/abs(ego_max_dece);
if t <= t_dece
    % 匀减速
    s_min = 0 + plan_start_s_dot * t + 0.5 * ego_max_dece * t^2;
end

if t >= t_dece
    s_min = 0 + plan_start_s_dot * t_dece + 0.5 * ego_max_dece * t_dece^2;
end
s_max = min(s_max, 58);
if s_max - s_min >= 0.2
    valid = 1;
end

end




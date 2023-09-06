function [filter_static_objects] = static_objects_filter(ego, static_objects)
% 该函数将筛选障碍物，纵向[-10,60] 横向[-10,10]的障碍物才会被考虑
% 该函数只是一种单车道的临时办法，考虑到多车道情况，即使障碍物距离较远也应该考虑
% EM Planner完全体是多车道并行计算的，每个车道都生成参考线然后并行计算出多条轨迹，再选择最优的轨迹作为输出
object = struct('valid',0,'x',0,'y',0,'v',0,'a',0,'length',0,'width',0,'heading',0);
filter_static_objects = repmat(object,10,1);%静态障碍物默认10个
count = 1;
for i = 1:length(static_objects)
    if static_objects(i).valid == 0
        continue;
    end
    %自车的heading的方向向量与法向量
    tor = [cos(ego.heading);sin(ego.heading)];
    nor = [-sin(ego.heading);cos(ego.heading)];
    %障碍物与自车的距离向量
    vector_obs = [static_objects(i).x;static_objects(i).y] - [ego.x;ego.y];
    %障碍物纵向距离
    lon_distance = vector_obs'*tor;
    %障碍物横向距离
    lat_distance = vector_obs'*nor;
    
    if (lon_distance < 60) && (lon_distance > -10) && (lat_distance > -10) && (lat_distance < 10)
        filter_static_objects(count).x = static_objects(i).x;
        filter_static_objects(count).y = static_objects(i).y;
        filter_static_objects(count).heading = static_objects(i).heading;
        filter_static_objects(count).v = static_objects(i).v;
        filter_static_objects(count).length = static_objects(i).length;
        filter_static_objects(count).width = static_objects(i).width;
        filter_static_objects(count).valid = 1;
        count = count + 1;
    end
end
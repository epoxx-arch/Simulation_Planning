function [match_point_index,proj_x, proj_y,proj_heading,proj_kappa] = matchpoint(...
    x,y,path_x,path_y,path_heading,path_kappa)

match_point_index = 0;
% 声明increase_count，用于表示在遍历时distance连续增加的个数
increase_count = 0;
% 开始遍历
min_distance = inf;
for j = 1 : length(path_x)
    distance = (x - path_x(j))^2 + (y - path_y(j))^2;
    if distance < min_distance
        min_distance = distance;
        match_point_index = j;
        increase_count = 0;
    else
        increase_count = increase_count + 1;
    end
    %如果distance连续增加50次就不要再遍历了，节省时间
    if increase_count > 50
        break;
    end
end
%取出匹配点的信息
match_point_x = path_x(match_point_index);
match_point_y = path_y(match_point_index);
match_point_heading = path_heading(match_point_index);
match_point_kappa = path_kappa(match_point_index);
%计算匹配点的方向向量与法向量
vector_match_point = [match_point_x;match_point_y];
vector_match_point_direction = [cos(match_point_heading);sin(match_point_heading)];
%声明待投影点的位矢
vector_r = [x;y];

%通过匹配点计算投影点
vector_d = vector_r - vector_match_point;
ds = vector_d' * vector_match_point_direction;
vector_proj_point = vector_match_point + ds * vector_match_point_direction;
proj_heading = match_point_heading + match_point_kappa * ds;
proj_kappa = match_point_kappa;
%计算结果输出
proj_x = vector_proj_point(1);
proj_y = vector_proj_point(2);
end


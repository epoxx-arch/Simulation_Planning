function index2s  = get_referenceline_s(path_x,path_y,origin_x,origin_y,origin_match_point_index)
%该函数将输出index与s的转换关系，index2s表示当path_x,path_y的离散点的编号为i时，对应的弧长为index2s(i)
%输入 path_x path_y 待转化的离散点的集合
%     origin_x,y frenet原点在世界坐标系下的坐标
%     origin_match_point_index 原点的匹配点的编号
% path点的个数
n = length(path_x);
% 输出初始化
index2s = zeros(n,1);
% 首先计算以path起点为坐标原点的index2s
for i = 2:n
    index2s(i) = sqrt((path_x(i) - path_x(i-1))^2 + (path_y(i) - path_y(i-1))^2) + index2s(i-1);
end
% 再计算以轨迹起点到frenet_path的坐标原点的弧长，记为s0，再用index2s - s0 就是最终的结果
% 计算s0
% s_temp frenet原点的匹配点的弧长
s_temp = index2s(origin_match_point_index);
%判断原点在匹配点的前面还是后面
%两个向量match_point_to_origin 
%        match_point_to_match_point_next
vector_match_2_origin = [origin_x;origin_y] - [path_x(origin_match_point_index);path_y(origin_match_point_index)];
vector_match_2_match_next = [path_x(origin_match_point_index + 1);path_y(origin_match_point_index + 1)] - ...
                            [path_x(origin_match_point_index);path_y(origin_match_point_index)];
if vector_match_2_origin'*vector_match_2_match_next > 0
    %坐标原点在匹配点的前面
    s0 = s_temp + sqrt(vector_match_2_origin'*vector_match_2_origin);
else
    %坐标原点在匹配点的后面
    s0 = s_temp - sqrt(vector_match_2_origin'*vector_match_2_origin);
end

index2s = index2s - ones(n,1)*s0;















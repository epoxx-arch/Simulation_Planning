function [referenceline_x_init,referenceline_y_init,referenceline_heading_init,referenceline_kappa_init] = get_plan_referenceline(...
host_match_point_index,global_path_x,global_path_y,global_path_heading,global_path_kappa)
%该函数将在全局路径上提取referenceline的未平滑的初值
%输入： host_match_point_index 自车的位置在全局路径的匹配点的编号
       %global_path_x,global_path_y 全局路径数据
%输出：referenceline_x_init,referenceline_y_init 未平滑的参考线的xy坐标

%由于global path 是每1m取一个点，所以从匹配点往后取30个点，往前取150个点即可，一共181个点
%后面的点不够的话就用前面的点补，前面的点不够的话就用后面的点补，保证总数是181个

%索引初始化
start_index = -1;
%判断后面前面的点是否足够多
if host_match_point_index - 5 < 1
    %匹配点后面的点太少了，不够30个
    start_index = 1;
elseif host_match_point_index + 20 > length(global_path_x)
    %匹配点前面的点太少了，不够150个
    start_index = length(global_path_x) - 20;
    
else
    %匹配点正常情况
    start_index = host_match_point_index - 5;
end
%提取referenceline
referenceline_x_init = global_path_x(start_index:start_index + 40);
referenceline_y_init = global_path_y(start_index:start_index + 40);
referenceline_heading_init = global_path_heading(start_index:start_index + 40);
referenceline_kappa_init = global_path_kappa(start_index:start_index + 40);
end


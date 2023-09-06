function [trajectory_s_end,trajectory_index2s] = path_trajectory_sum_s(trajectory_x,trajectory_y)
% 该函数将计算以trajectory的s 与 x y 的对应关系，可以看作是trajectory index2s
n = length(trajectory_x);
trajectory_index2s = zeros(n,1);
sum = 0;
for i = 2:length(trajectory_x)
    if isnan(trajectory_x(i))
        break;
    end
    sum = sum + sqrt((trajectory_x(i) - trajectory_x(i-1))^2 + (trajectory_y(i) - trajectory_y(i-1))^2);
    trajectory_index2s(i) = sum;
end
% 计算出trajectory的长度
if i == n
    trajectory_s_end = trajectory_index2s(end);
else
    % 因为循环的退出条件为isnan(trajectory_x(i)) 所以 i 所对应的数为 nan 
    trajectory_s_end = trajectory_index2s(i - 1);
end

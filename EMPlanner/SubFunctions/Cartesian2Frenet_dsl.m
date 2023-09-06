function [s_dot_set,l_dot_set,dl_set] = Cartesian2Frenet_dsl(l_set,vx_set,vy_set,proj_heading_set,proj_kappa_set)
%该函数将计算frenet坐标系下的s_dot, l_dot, dl/ds
n = length(vx_set);
% 输出初始化
s_dot_set = ones(n,1)*nan;
l_dot_set = ones(n,1)*nan;
dl_set = ones(n,1)*nan;

for i = 1 : length(l_set)
    if isnan(l_set(i))
        break;
    end
    v_h = [vx_set(i);vy_set(i)];
    n_r = [-sin(proj_heading_set(i));cos(proj_heading_set(i))];
    t_r = [cos(proj_heading_set(i));sin(proj_heading_set(i))];
    l_dot_set(i) = v_h'*n_r;
    s_dot_set(i) = v_h'*t_r/(1 - proj_kappa_set(i)*l_set(i));
    %%%向量法做cartesian与frenet的转换要更简单，但是也有缺点，向量法必须依赖速度加速度
    %l' = l_dot/s_dot 但是如果s_dot = 0 此方法就失效了
    if abs(s_dot_set(i)) < 1e-6
        dl_set(i) = 0;
    else
        dl_set(i) = l_dot_set(i)/s_dot_set(i);
    end
end



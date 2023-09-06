function [s_dot2_set,l_dot2_set,ddl_set] = Cartesian2Frenet_ddsl(ax_set,ay_set,proj_heading_set,proj_kappa_set,l_set,s_dot_set,dl_set)
%由于不知道有多少个点需要做坐标转换，设一个最大值做缓冲
n = length(ax_set);
% 输出初始化
s_dot2_set = ones(n,1)*nan;
l_dot2_set = ones(n,1)*nan;
ddl_set = ones(n,1)*nan;

for i = 1:length(l_set)
    if isnan(l_set(i))
       break;
    end
    a_h = [ax_set(i);ay_set(i)];
    n_r = [-sin(proj_heading_set(i));cos(proj_heading_set(i))];
    t_r = [cos(proj_heading_set(i));sin(proj_heading_set(i))];
    %近似认为dkr/ds 为0 简化计算
    l_dot2_set(i) = a_h'*n_r - proj_kappa_set(i) * (1 - proj_kappa_set(i) * l_set(i)) * s_dot_set(i)^2;
    s_dot2_set(i) = (1/(1 - proj_kappa_set(i) * l_set(i)))* (a_h' * t_r + 2 * proj_kappa_set(i) * dl_set(i) * s_dot_set(i)^2);
    % 要考虑加速度为0的情况
    if (s_dot2_set(i) < 1e-6)
        ddl_set(i) = 0;
    else
        ddl_set(i) = (l_dot2_set(i) - dl_set(i)*s_dot2_set(i))/(s_dot_set(i)^2);
    end
end

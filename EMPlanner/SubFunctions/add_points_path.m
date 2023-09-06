function [qp_path_s_final,qp_path_l_final, qp_path_dl_final,qp_path_ddl_final] =...
    add_points_path(qp_path_s,qp_path_l,qp_path_dl,qp_path_ddl)
% 该函数将增密qp_path
n_init = 60;
% 增密的点的个数
n = 120;
% 输出初始化
qp_path_s_final = zeros(n,1);
qp_path_l_final = zeros(n,1);
qp_path_dl_final = zeros(n,1);
qp_path_ddl_final = zeros(n,1);
ds = (qp_path_s(end) - qp_path_s(1))/(n-1);
index = 1;
for i = 1:n
    x = qp_path_s(1) + (i-1) * ds;
    qp_path_s_final(i) = x;
    while x >= qp_path_s(index)
        index = index + 1;
        if index == n_init
            break;
        end
    end
    % while 循环退出的条件是x<qp_path_s(index)，所以x对应的前一个s的编号是index-1 后一个编号是index
    pre = index - 1;
    cur = index;
    % 计算前一个点的l l' l'' ds 和后一个点的 l''
    delta_s = x - qp_path_s(pre);
    l_pre = qp_path_l(pre);
    dl_pre = qp_path_dl(pre);
    ddl_pre = qp_path_ddl(pre);
    ddl_cur = qp_path_ddl(cur);
    % 分段加加速度优化 
    qp_path_l_final(i) = l_pre + dl_pre * delta_s + (1/3)* ddl_pre * delta_s^2 + (1/6) * ddl_cur * delta_s^2;
    qp_path_dl_final(i) = dl_pre + 0.5 * ddl_pre * delta_s + 0.5 * ddl_cur * delta_s;
    qp_path_ddl_final(i) = ddl_pre + (ddl_cur - ddl_pre) * delta_s/(qp_path_s(cur) - qp_path_s(pre));
    %  因为此时x的后一个编号是index 必有x < qp_path_s(index),在下一个循环中x = x + ds 也未必大于
    %  qp_path_s(index)，这样就进入不了while循环，所以index 要回退一位
    index = index - 1;   
end
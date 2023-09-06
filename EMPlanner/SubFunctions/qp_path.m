function [qp_path_s, qp_path_l, qp_path_dl, qp_path_ddl] = qp_path(...
    ego_plan_start_s, ego_plan_start_l, ego_plan_start_dl, ego_plan_start_ddl,...
    l_max, l_min)

%规划的起点
plan_start_s = ego_plan_start_s;
plan_start_l = ego_plan_start_l;
plan_start_dl = ego_plan_start_dl;
plan_start_ddl = ego_plan_start_ddl;
%% 路径二次规划
% 0.5*x'Hx + f'*x = min
% subject to A*x <= b
%            Aeq*x = beq
%            lb <= x <= ub;
% 输入：l_min l_max 点的凸空间
%       w_cost_l 参考线代价
%       w_cost_dl ddl dddl 光滑性代价
%       w_cost_centre 凸空间中央代价
%       w_cost_end_l dl dd1 终点的状态代价 (希望path的终点状态为(0,0,0))
%       host_d1,d2 host质心到前后轴的距离
%       host_w host的宽度
%       plan_start_l,dl,ddl 规划起点
% 输出 qp_path_l dl ddl 二次规划输出的曲线
%惩罚参数 w_cost_l 靠近中心线 
%惩罚参数 w_cost_centre 靠近凸空间
%参数w_cost_dl 横向速度
w_cost_l = 2; w_cost_dl = 25000; w_cost_ddl = 50;w_cost_dddl = 20;
w_cost_centre = 1500;
w_cost_end_l = 15;w_cost_end_dl = 15;w_cost_end_ddl = 15;
host_d1 = 2; host_d2 = 2; host_w = 1.6;
delta_dl_max = 2; delta_ddl_max = 1;

% 待优化的变量的数量
n = 60;
% 输出初始化
qp_path_l = zeros(n,1);
qp_path_dl = zeros(n,1);
qp_path_ddl = zeros(n,1);
qp_path_s = zeros(n,1);
% H_L H_DL H_DDL H_DDDL Aeq beq A b 初始化
H_L = zeros(3*n, 3*n);
H_DL = zeros(3*n, 3*n);
H_DDL = zeros(3*n, 3*n);
H_DDDL = zeros(n-1, 3*n);
H_CENTRE = zeros(3*n, 3*n);
H_L_END = zeros(3*n, 3*n);
H_DL_END = zeros(3*n, 3*n);
H_DDL_END = zeros(3*n, 3*n);
Aeq = zeros(2*n-2, 3*n);%连续性等式约束
beq = zeros(2*n-2, 1);
A = zeros(8*n, 3*n);%凸空间的约束
b = zeros(8*n, 1);%
% 更新：加入 dl(i+1) - dl(i) ddl(i+1) - ddl(i) 的约束
A_dl_minus = zeros(n - 1,3*n);
b_dl_minus = zeros(n - 1,1);
A_ddl_minus = zeros(n - 1,3*n);
b_ddl_minus = zeros(n - 1,1);
for i = 1:n-1
    row = i;
    col = 3*i - 2;
    A_dl_minus(row,col:col+5) = [0 -1 0 0 1 0];
    b_dl_minus(row) = delta_dl_max;
    A_ddl_minus(row,col:col+5) = [0 0 -1 0 0 1];
    b_ddl_minus(row) = delta_ddl_max;
end
% -max < a*x < max => ax < max && -ax < -(-max)
A_minus = [A_dl_minus;
          -A_dl_minus;
           A_ddl_minus;
          -A_ddl_minus];%暂时没用
      
b_minus = [b_dl_minus;
           b_dl_minus;
          b_ddl_minus;
          b_ddl_minus]; %暂时也没用

%  期望的终点状态
end_l_desire = 0;
end_dl_desire = 0;
end_ddl_desire = 0;

% Aeq_sub 连续性约束
ds = 1;%纵向间隔 default 3米
% plan_start_s = 0 n = 10
%[0 3 6 9 12 15 ... 57]
for i = 1:n
    qp_path_s(i) = plan_start_s + (i-1)*ds;
end

Aeq_sub = [1 ds ds^2/3 -1 0 ds^2/6;
           0 1  ds/2   0 -1 ds/2];%2行6列 连续性子矩阵
% A_sub;
d1 = host_d1;%自车的中心到前轴
d2 = host_d2;%自车中心到后轴
w = host_w;%自车的宽
A_sub = [1  d1 0;
         1  d1 0;
         1 -d2 0;
         1 -d2 0;
        -1 -d1 0;
        -1 -d1 0;
        -1  d2 0;
        -1  d2 0];%8行3列 自车四个角点约束
    
% 生成Aeq 38行60列
for i = 1:n-1
    % 计算分块矩阵左上角的行和列
    row = 2*i - 1;%[1 3 5 7 ...37]
    col = 3*i - 2;%[1 4 7 ... ]
    Aeq(row:row + 1,col:col + 5) = Aeq_sub;
    %Aeq_sub2行6列
    %row:row + 1 和 col:col + 5 = 1到2行且1到6列
    %3到4行且4到9列 ... 37到38行且55列到60列
end
% 生成A -> 160*60 凸空间的不等式约束
%A_sub 8行3列 20个点
for i = 2:n
    row = 8*i - 7;%[1 9 ... ]
    col = 3*i - 2;%[1 4 ... ]
    A(row:row + 7,col:col + 2) = A_sub;
    %第一个自车四个角点的凸空间约束的子矩阵放入 1到8行且1到3列
    %第一个自车四个角点的凸空间约束的子矩阵放入 9到16行且4到6列 ...
end

% 视频里用的找(s(i) - d2,s(i) + d1)的方法过于保守，在这里舍弃掉
% 只要找到四个角点所对应的l_min l_max 即可
% 。。。。。。。。。。。。。。
%    [    .   ]<- 
%    [        ]
% 。。。。。。。。。。。。。
%自车中心到前后轴心都是2米 ds1米 因此front_index = 2 back_index = 2
front_index = ceil(d1/ds); % d1车头位置 d2车尾位置
back_index = ceil(d2/ds);  % ceil向上取整 ceil(3) = 3 ceil(3.1) = 4
% 生成b 自车四个角点的凸空间约束 A*X < b
for i = 2:n
    % 左前 右前的index = min(i + front_index,60)
    % 左后 右后的index = max(i - back_index,1)
    % l_min l_max 都是60个点
    % l的max和min是一米一个，ds是三米一个，因此采样[1 4 7 ]
    % dp_s 从规划的起点向前80米， n=60,因此不会超
    index1 = i + front_index + 1;
    index2 = max(i - back_index + 1,1);
    b(8*i - 7:8*i,1) = [l_max(index1) - w/2; %l_max 左边界
                        l_max(index1) + w/2;
                        l_max(index2) - w/2;
                        l_max(index2) + w/2;
                       -l_min(index1) + w/2; %l_min 右边界
                       -l_min(index1) - w/2;
                       -l_min(index2) + w/2;
                       -l_min(index2) - w/2;];
end

%生成 lb ub 主要是对规划起点做约束
% lb < x < ub
lb = ones(3*n,1)*-inf;
ub = ones(3*n,1)*inf;
lb(1) = plan_start_l;%起点的约束
lb(2) = plan_start_dl;
lb(3) = plan_start_ddl;
ub(1) = lb(1);%起点等式约束
ub(2) = lb(2);
ub(3) = lb(3);
for i = 2:n
    lb(3*i - 1) = - 0.2; %约束 l'
    ub(3*i - 1) = 0.2;
    lb(3*i) = -0.02; %约束 l''
    ub(3*i) = 0.02;
end

% 生成H_L,H_DL,H_DDL,H_CENTRE
for i = 1:n
    H_L(3*i - 2,3*i - 2) = 1;
    H_DL(3*i - 1,3*i - 1) = 1;
    H_DDL(3*i, 3*i) = 1;
end
H_CENTRE = H_L;
% 生成H_DDDL;
H_dddl_sub = [0 0 1 0 0 -1];%1行且6列
for i = 1:n-1
    row = i;%      [1 2 3 .. 19]
    col = 3*i - 2;%[1 4 7 .. ]
    H_DDDL(row,col:col + 5) = H_dddl_sub;
    %第一个子矩阵放入 1行且1到6列
    %第二个子矩阵放入 2行且7到12列
    %  19行且55到60列
end
% 生成H_L_END H_DL_END H_DDL_END 最后一个点
H_L_END(3*n - 2,3*n - 2) = 1;% 58行且58列 
H_DL_END(3*n - 1,3*n - 1) = 1;% 59行且59列
H_DDL_END(3*n,3*n) = 1;% 60行且60列
% 生成二次规划的H 因为ds ！= 1 所以 dddl = delta_ddl/ds;
H = w_cost_l * (H_L'*H_L) + w_cost_dl * (H_DL'*H_DL) + w_cost_ddl * (H_DDL'*H_DDL)...
   + w_cost_dddl * (H_DDDL'*H_DDDL)/(ds*ds) + w_cost_centre * (H_CENTRE'*H_CENTRE) + w_cost_end_l * (H_L_END'*H_L_END)...
   + w_cost_end_dl * (H_DL_END'* H_DL_END) + w_cost_ddl * (H_DDL_END'*H_DDL_END);
H = 2 * H;%H是60行且60列
% 生成f
f = zeros(3*n,1);% l的max和min是一米一个，ds是三米一个，因此采样。
centre_line = 0.5 * (l_min + l_max); % 此时centre line 还是60个点
% centre_line = dp_path_l_final;
for i = 1:n
%     f(3*i - 2) = -2 * centre_line(3*i - 2);
    f(3*i - 2) = -2 * centre_line(i);% ds = 1
end
% 避免centreline权重过大影响轨迹平顺性
for i = 1:n
    if abs(f(i)) > 0.01
        f(i) = w_cost_centre * f(i);
    end
end
% 终点要接近end_l dl ddl desire
f(3*n - 2) = f(3*n - 2) -2 * end_l_desire * w_cost_end_l;
f(3*n - 1) = f(3*n - 1) -2 * end_dl_desire * w_cost_end_dl;
f(3*n) = f(3*n) -2 * end_ddl_desire * w_cost_end_ddl;

X = quadprog(H,f,A,b,Aeq,beq,lb,ub);

for i = 1:n
    qp_path_l(i) = X(3*i - 2);
    qp_path_dl(i) = X(3*i - 1);
    qp_path_ddl(i) = X(3*i);
end

end
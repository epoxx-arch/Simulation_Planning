
%% lookup for target acc
%自车的状态
init_ego_s = 0;
init_ego_v = 25;
init_ego_a = 0;
cur_ego_s = init_ego_s;
cur_ego_v = 25;
cur_ego_a = 0;
%前车的状态
init_front_s = 50;
init_front_v = 10;
init_front_a = 0;
% simulation
num = 100;
dt = 0.1;
Acc_arr = zeros(1, num);
V_arr = zeros(1, num);
Delta_s_arr = zeros(1, num);
Delta_v_arr = zeros(1, num);
pos_jerk = 2;
neg_jerk = -2;
delta_a = 0.2;
pre_acc = 0;
for i = 1:num
cur_front_v =  init_front_v;%匀速直线运动   
cur_front_s =  init_front_s + cur_front_v * (i-1)*dt;

delta_s = (cur_front_s - cur_ego_s) - (cur_front_v * 2.0);
Delta_s_arr(i) = delta_s;
delta_v = cur_ego_v - cur_front_v;
Delta_v_arr(i) = delta_v;
%二维查表
acc = interp2(DeltaVArr, DeltaSArr, AccReqTable, delta_v, delta_s);
%jerk约束
if acc > pre_acc + 0.2
    acc = pre_acc + 0.2;
end
if acc < pre_acc - 0.2
    acc = pre_acc - 0.2;
end
if acc <= -4
    acc = -4;
end
pre_acc = acc;
cur_ego_a = cur_ego_a * 0.6 + acc * 0.4;
cur_ego_v = cur_ego_v + dt * acc;
cur_ego_s = cur_ego_s + cur_ego_v * dt + 0.5 * dt^2 * cur_ego_a;

Acc_arr(i) = acc;
V_arr(i) = cur_ego_v;
end

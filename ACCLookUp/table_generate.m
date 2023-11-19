%% 分区域 减速区

%% 第四象限
% for i = 1:11
% delta_s = 2;
% delta_v = 30 - (i-1)*2;
% acc_req = -delta_v^2/(2 * delta_s);
% acc_req = roundn(acc_req, -2);
% acc_req_arr(i) = acc_req;
% end
% 
% for i = 1:9
% delta_s = 2;
% delta_v = 9 - (i-1);
% acc_req = -delta_v^2/(2 * delta_s);
% acc_req = roundn(acc_req, -2);
% acc_req_arr_1(i) = acc_req;
% end

% %% 第一象限
% %delta_v > 0 自车车速 大于 前车车速
% %delta_s < 0 目标位置在自车位置后面
% acc_req_arr = zeros(1, 11);
% acc_req_arr_1 = zeros(1, 9);
% p0 = 0.4;
% p1 = -0.5;
% % -2 到 -14 p0=0.2 大于18 p0 0.3 大于-25 p0 0.4
% for i = 1:11
% delta_s = -25;
% delta_v = 30 - (i-1)*2;
% 
% acc_req = delta_s * p0 + delta_v * p1;
%  if acc_req < -10
%      acc_req = -10.00;
%  end
%  if acc_req > 10
%      acc_req = 10.00;
%  end
%  
% acc_req = roundn(acc_req, -2);
% acc_req_arr(i) = acc_req;
% end
% 
% for i = 1:9
% delta_s = -25;
% delta_v = 9 - (i-1);
% 
% acc_req = delta_s * p0 + delta_v * p1;
%  if acc_req < -10
%      acc_req = -10.00;
%  end
%  if acc_req > 10
%      acc_req = 10.00;
%  end
% acc_req = roundn(acc_req, -2);
% acc_req_arr_1(i) = acc_req;
% end

% %% 第三象限
% %delta_v < 0 自车车速 小于 前车车速
% %delta_s > 0 目标位置在自车位置 前面  因此加速
% %delta_v -> p1  
% acc_req_arr = zeros(1, 11);
% acc_req_arr_1 = zeros(1, 9);
% p0 = 0.030;
% p1 = -0.3;
% %delta_s[-1 -2]->p0= -0.1  [-3 -8]->0.08
% for i = 1:11
% delta_s = 90;
% delta_v = -(30 - (i-1)*2);
% 
% acc_req = delta_s * p0 + delta_v * p1;
%  if acc_req < -10
%      acc_req = -10.00;
%  end
%  if acc_req > 4
%      acc_req = 4.00;
%  end
%  
% acc_req = roundn(acc_req, -2);
% acc_req_arr(i) = acc_req;
% end
% 
% for i = 1:9
% delta_s = 90;
% delta_v = -(9 - (i-1));
% 
% acc_req = delta_s * p0 + delta_v * p1;
%  if acc_req < -10
%      acc_req = -10.00;
%  end
%  if acc_req > 4
%      acc_req = 4.00;
%  end
% acc_req = roundn(acc_req, -2);
% acc_req_arr_1(i) = acc_req;
% end

%% 第二象限 加减速不确定区域
%delta_v > 0 自车车速 小于 前车车速
%delta_s > 0 目标位置在自车位置后  因此加速
%delta_v -> p1  
acc_req_arr = zeros(1, 11);
acc_req_arr_1 = zeros(1, 9);
p0 = 0.18;
p1 = -0.3;
%delta_s[-1 -2]->p0= -0.1  [-3 -8]->0.08
for i = 1:11
delta_s = -40;
delta_v = -(30 - (i-1)*2);
acc_req = delta_s * p0 + delta_v * p1;
 if acc_req < -10
     acc_req = -10.00;
 end
 if acc_req > 4
     acc_req = 4.00;
 end
 
acc_req = roundn(acc_req, -2);
acc_req_arr(i) = acc_req;
end

for i = 1:9
delta_s = -40;
delta_v = -(9 - (i-1));

acc_req = delta_s * p0 + delta_v * p1;
 if acc_req < -10
     acc_req = -10.00;
 end
 if acc_req > 4
     acc_req = 4.00;
 end
acc_req = roundn(acc_req, -2);
acc_req_arr_1(i) = acc_req;
end


%% ������ ������

%% ��������
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

% %% ��һ����
% %delta_v > 0 �Գ����� ���� ǰ������
% %delta_s < 0 Ŀ��λ�����Գ�λ�ú���
% acc_req_arr = zeros(1, 11);
% acc_req_arr_1 = zeros(1, 9);
% p0 = 0.4;
% p1 = -0.5;
% % -2 �� -14 p0=0.2 ����18 p0 0.3 ����-25 p0 0.4
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

% %% ��������
% %delta_v < 0 �Գ����� С�� ǰ������
% %delta_s > 0 Ŀ��λ�����Գ�λ�� ǰ��  ��˼���
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

%% �ڶ����� �Ӽ��ٲ�ȷ������
%delta_v > 0 �Գ����� С�� ǰ������
%delta_s > 0 Ŀ��λ�����Գ�λ�ú�  ��˼���
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


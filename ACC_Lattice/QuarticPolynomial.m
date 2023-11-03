%% 定速巡航的轨迹生成
clear all;
%% 自车状态和目标速度
%自车状态
ego_s0 = 10;
ego_v = 80/3.6;
ego_a = 2;
%终点状态
end_v = 60/3.6;% 巡航的车速
end_a = 0;
end_time = 3;

%% 采样的方式
% 对末点的采样规则：时距为8s，采样间隔1s，所以实际的采样点相对时间戳为：[0.01, 1, 2, 3, 4...7, 8]s。
% 每个采样时刻进行6次采样，根据车辆的加、减速度最大值（更像是非剧烈驾驶下的最大值）以及最大巡航速度，
% 计算该时刻车速的范围，然后均匀插入4个点 
coef_ = QuarticPolynomialComputeCoefficients(ego_s0, ego_v, ego_a,  end_v, end_a,  end_time);
 [JerkMax, JerkMin] = CalcJerkRangeOfQuartic(coef_, end_time, 0);
 [AccMax, AccMin] = CalcAccRangeOfQuartic(coef_, end_time, 0);
 [VMax, VMin] = CalcVRangeOfQuartic(coef_,  end_time, 0);
%% figure trajectory
tra_result = true;
tra_time = end_time;
tra_coef = coef_;
if tra_result == true
    time_arr = [];
    position_arr = [];
    velocity_arr = [];
    accelerate_arr = [];
    jerk_arr = [];
    tra_num = tra_time/0.1; % 0.1 get one point
    for i = 1:tra_num
        time_arr(i) = i*0.1;
        position_arr(i) = QuarticPolynomialEvaluate(tra_coef, 0, i*0.1);
        velocity_arr(i) = QuarticPolynomialEvaluate(tra_coef, 1, i*0.1);
        accelerate_arr(i) = QuarticPolynomialEvaluate(tra_coef, 2, i*0.1);
        jerk_arr(i) = QuarticPolynomialEvaluate(tra_coef, 3, i*0.1);    
    end
    cut_in_s = position_arr(tra_num);
    figure(1);
    plot(time_arr,position_arr);
    title('距离S');
    figure(2);
    plot(time_arr,velocity_arr);
    title('速度V');
    figure(3);
    plot(time_arr,accelerate_arr);
    title('加速度A');
    figure(4);
    plot(time_arr,jerk_arr);
    title('冲击度jerk');
end

%% 四次多项式 get value from s = f(t) quintic polynomial
% order derivation
% p time of all
function value = QuarticPolynomialEvaluate(tra_coef, order, p)
    value = 0;
    switch (order) 
        case 0 % position
            value = (((tra_coef(5) * p + tra_coef(4)) * p + tra_coef(3)) * p + ...
                tra_coef(2)) * p + tra_coef(1);
        case 1 % velocity
            value = ((4.0 * tra_coef(5) * p + 3.0 * tra_coef(4)) * p + ...
                2.0 * tra_coef(3)) * p + tra_coef(2);
        case 2 % accelerate
            value = (( 12.0 * tra_coef(5)) * p + 6.0 * tra_coef(4)) * p + ...
             2.0 * tra_coef(3);
        case 3 %jerk
            value =  24.0 * tra_coef(5) * p + 6.0 * tra_coef(4);
        case 4 % djerk
            value = 24.0 * tra_coef(5);
    end
end

%四次多项式计算系数
%x0 = s0 dx0 = s_dot ddx0 = s_dotdot 自车当前的位置 速度 加速度
%dx1 是目标速度 ddx1目标加速度
% p = ti 时间系数
function coef = QuarticPolynomialComputeCoefficients(x0, dx0, ddx0,  dx1, ddx1,  p) 
    coef(1) = x0;
    coef(2) = dx0;
    coef(3) = ddx0 / 2.0;
    p2 = p * p;
    p3 = p * p2;
    b0 = dx1 - ddx0 * p - dx0;
    b1 = ddx1 - ddx0;
    coef(4) = (3 * b0 - b1 * p) / (3 * p2);
    coef(5) = (-2 * b0 + b1 * p) / (4 * p3);
end

%% calculate jerk max min 
% Coef 四次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 
% Tmin_ 五次多项式开始的时间 Tmax_ 五次多项式结束的时间
% JerkMax, JerkMin计算出最大最小的jerk
function [JerkMax, JerkMin] = CalcJerkRangeOfQuartic(Coef, Tmax_, Tmin_)
    % ax + b = 0 jerk极值的两个端点
    jerk_1 = QuarticPolynomialEvaluate(Coef, 3, Tmin_);
    jerk_2 = QuarticPolynomialEvaluate(Coef, 3, Tmax_);
    JerkMax = max(jerk_1, jerk_2);
    JerkMin = min(jerk_1, jerk_2);
end

%% calculate Acc max min and zero point t0 and t1
% Coef 四次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 
% Tmin_ 四次多项式开始的时间 Tmax_ 四次多项式结束的时间
% AccMax, AccMin 计算出最大最小的加速度
function [AccMax, AccMin] = CalcAccRangeOfQuartic(Coef, Tmax_, Tmin_)
    % acc equation ax^2 + bx + c = 0; jerk equation ax + b = 0
    %最大的jerk可能在最大的端点
    acc_1 = QuarticPolynomialEvaluate(Coef, 2, Tmin_);
    acc_2 = QuarticPolynomialEvaluate(Coef, 2, Tmax_);
    AccMax = max(acc_1, acc_2);
    AccMin = min(acc_1, acc_2);
    %根据零点计算
    a = 24.0 * Coef(5);
    b = 6.0 * Coef(4);
    if b ~= 0
        x = -b/a;
        if x >= Tmin_ && x <= Tmax_
            acc = QuarticPolynomialEvaluate(Coef, 2, x);
            AccMax = max(AccMax, acc);
            AccMin = min(AccMin, acc);
        end  
    end
end

%% calculate V max min
% Coef 四次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 
% Tmin_ 四次多项式开始的时间 Tmax_ 五次多项式结束的时间
% VMax, VMin 计算出最大最小的加速度
function [VMax, VMin] = CalcVRangeOfQuartic(Coef, Tmax_, Tmin_)
    %计算两端的加速度值
    v0 = QuarticPolynomialEvaluate(Coef, 1, Tmin_);
    v1 = QuarticPolynomialEvaluate(Coef, 1, Tmax_);
    VMax = max(v0, v1);
    VMin = min(v0, v1);
    %计算零点的个数
    %jerk equation ax^2 + bx + c = 0;
    a = 12.0 * Coef(5);
    b = 6.0 * Coef(4);
    c = 2.0 * Coef(3);
    delta = b * b - 4 * a * c;
    if a == 0
        if b ~= 0
            %有一个零点
             x = -b/a;
            if x >= Tmin_ && x <= Tmax_
                v = QuarticPolynomialEvaluate(Coef, 1, x);
                VMax = max(VMax, v);
                VMin = min(VMin, v);
            end
        end
    else
        if delta > 0
            %两个零点
            x1 = (-b - sqrt(delta))/(2 * a);
            if x1 >= Tmin_ && x1 <= Tmax_
                v_1 = QuarticPolynomialEvaluate(Coef, 1, x1);
                VMax = max(VMax, v_1);
                VMin = min(VMin, v_1);
            end 
            x2 = (-b + sqrt(delta))/(2 * a);
            if x2 >= Tmin_ && x2 <= Tmax_
                v_2 = QuarticPolynomialEvaluate(Coef, 1, x2);
                VMax = max(VMax, v_2);
                VMin = min(VMin, v_2);
            end 
        elseif delta == 0
                %一个零点
                 x = -b / (2*a);
                if x >= Tmin_ && x <= Tmax_
                  v = QuarticPolynomialEvaluate(Coef, 1, x);
                  VMax = max(VMax, v);
                  VMin = min(VMin, v);
                end
        else
            %没有零点
        end
    end
end





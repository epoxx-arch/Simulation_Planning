clear all;
%% 跟随前车

%% 约束限制和权重
JerkWeight = 10;
TimeWeight = 2;
SpeedWeight = 5;

JerkLimitMax = 3;
JerkLimitMin = -5;
AccLimitMax = 2.0;
AccLimitMin = -5.5;
VLimitMax = 30;
VLimitMin = 0;
TotalTime = 16;


kJerkRangeY = [0 0 0.2 0.8 2];
kJerkRangeX = [0 1  2   3  4];

%% init paras
tra_score = 100;
tra_jerk_min = 4;% default
tra_result = false;
tra_time = 30;
tra_coef_quintic = zeros(6,1);
tra_coef_quartic = zeros(5,1);
tra_type = 1; %1是五次多项式  2四次多项式
%% 自车状态和目标速度
%自车状态
ego_s0 = 0;
ego_v = 80  /3.6;
ego_a = 0.0;
%前车的状态
front_s1 = 100;
front_v = 60/3.6;
front_a = -0.8;
% end time 时间后
target_v = front_v;
target_a = 0;

%% 采样的方式 五次多项式
for ti = 0.5 : 0.5 : TotalTime
    target_s = front_s1 + front_v * ti + 0.5 * front_a * ti^2 - front_v * 1.5;% 目标位置
    target_v = front_v + front_a * ti; % 目标车速
    target_a = 0; % 目标加速度
    coef_ = QuinticPolynomialComputeCoefficients(ego_s0, ego_v, ego_a,  target_s, target_v, target_a, ti);
    [JerkMax, JerkMin] = CalcJerkRangeOfTrajectory(coef_, ti, 0);
    [AccMax, AccMin] = CalcAccRangeOfTrajectory(coef_, ti, 0);
    [VMax, VMin] = CalcVRangeOfTrajectory(coef_,  ti, 0);
    if JerkMax <= JerkLimitMax && JerkMin >= JerkLimitMin && ...
        AccMax <= AccLimitMax && AccMin >= AccLimitMin && ...
        VMax <= VLimitMax && VMin >= VLimitMin
        tra_jerk_min = max(JerkMax, abs(JerkMin));%
        % calculate cost of trajectory include jerk
        speed_cost_sqr_sum = 0;
        speed_cost_weight_sum = 0;
        %随着时间增加 轨迹的速度追上目标速度
        for t = 0.5 : 0.5 : ti
            %当前时间前车的车速
            cur_front_v = front_v + front_a * ti; %匀速模型
            cur_ego_v = QuinticPolynomialEvaluate(coef_, 1, t);
            speed_cost_sqr_sum = speed_cost_sqr_sum + t*t  * abs(cur_ego_v - cur_front_v);
            speed_cost_weight_sum = speed_cost_weight_sum + t*t;
        end
        speed_cost = (speed_cost_sqr_sum/(speed_cost_weight_sum + 0.0001)) * SpeedWeight;
        %随着时间增加 轨迹的位置追上目标位置
        
        jerk_score = interp1(kJerkRangeX, kJerkRangeY, tra_jerk_min) * JerkWeight;
        time_score = (ti/TotalTime) * TimeWeight;
        sum_score = jerk_score + time_score + speed_cost;
        if sum_score <= tra_score
            tra_type = 1;
            tra_score = sum_score;
            tra_result = true;
            tra_coef_quintic = coef_;
            tra_time = ti;
            stop_end = 0;
        end
        %参数清零
        speed_cost = 0;
        speed_cost_sqr_sum = 0;
        speed_cost_weight_sum = 0;
    end
end

% 采样的方式 四次多项式
for ti = 0.5 : 0.5 : TotalTime
    target_v = front_v + front_a * ti; % 目标车速
    target_a = 0; % 目标加速度
    coef_ = QuarticPolynomialComputeCoefficients(ego_s0, ego_v, ego_a, target_v, target_a, ti);
     [JerkMax, JerkMin] = CalcJerkRangeOfQuartic(coef_, ti, 0);
     [AccMax, AccMin] = CalcAccRangeOfQuartic(coef_, ti, 0);
     [VMax, VMin] = CalcVRangeOfQuartic(coef_,  ti, 0);
    
    if JerkMax <= JerkLimitMax && JerkMin >= JerkLimitMin && ...
        AccMax <= AccLimitMax && AccMin >= AccLimitMin && ...
        VMax <= VLimitMax && VMin >= VLimitMin
        tra_jerk_min = max(JerkMax, abs(JerkMin));%
        % calculate cost of trajectory include jerk
        speed_cost_sqr_sum = 0;
        speed_cost_weight_sum = 0;
        %随着时间增加 轨迹的速度追上目标速度
        for t = 0.5 : 0.5 : ti
            %当前时间前车的车速
            cur_front_v = front_v + front_a * ti; %匀速模型
            cur_ego_v = QuarticPolynomialEvaluate(coef_, 1, t);
            speed_cost_sqr_sum = speed_cost_sqr_sum + t*t  * abs(cur_ego_v - cur_front_v);
            speed_cost_weight_sum = speed_cost_weight_sum + t*t;
        end
        speed_cost = (speed_cost_sqr_sum/(speed_cost_weight_sum + 0.0001)) * SpeedWeight;
        %随着时间增加 轨迹的位置追上目标位置
        
        jerk_score = interp1(kJerkRangeX, kJerkRangeY, tra_jerk_min) * JerkWeight;
        time_score = (ti/TotalTime) * TimeWeight;
        sum_score = jerk_score + time_score + speed_cost;
        if sum_score <= tra_score
            tra_type = 2;
            tra_score = sum_score;
            tra_result = true;
            tra_coef_quartic = coef_;
            tra_time = ti;
            stop_end = 0;
        end
        %参数清零
        speed_cost = 0;
        speed_cost_sqr_sum = 0;
        speed_cost_weight_sum = 0;
    end
end

%% figure trajectory
if tra_result == true
    time_arr = [];
    position_arr = [];
    velocity_arr = [];
    accelerate_arr = [];
    jerk_arr = [];
    tra_num = round(tra_time/0.1); % 0.1 get one point
    if tra_type == 1
         for i = 1:tra_num
            time_arr(i) = i*0.1;
            position_arr(i) = QuinticPolynomialEvaluate(tra_coef_quintic, 0, i*0.1);
            velocity_arr(i) = QuinticPolynomialEvaluate(tra_coef_quintic, 1, i*0.1);
            accelerate_arr(i) = QuinticPolynomialEvaluate(tra_coef_quintic, 2, i*0.1);
            jerk_arr(i) = QuinticPolynomialEvaluate(tra_coef_quintic, 3, i*0.1);    
        end
    else
         for i = 1:tra_num
            time_arr(i) = i*0.1;
            position_arr(i) = QuarticPolynomialEvaluate(tra_coef_quartic, 0, i*0.1);
            velocity_arr(i) = QuarticPolynomialEvaluate(tra_coef_quartic, 1, i*0.1);
            accelerate_arr(i) = QuarticPolynomialEvaluate(tra_coef_quartic, 2, i*0.1);
            jerk_arr(i) = QuarticPolynomialEvaluate(tra_coef_quartic, 3, i*0.1);    
        end
    end
   
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

%% get value from s = f(t) quintic polynomial
% order derivation
% p time of all
function value = QuinticPolynomialEvaluate(tra_coef, order, p)
    value = 0;
    switch (order) 
        case 0 % position
            value = ((((tra_coef(6) * p + tra_coef(5)) * p + tra_coef(4)) * p + tra_coef(3)) * p + ...
                tra_coef(2)) * p + tra_coef(1);
        case 1 % velocity
            value = (((5.0 * tra_coef(6) * p + 4.0 * tra_coef(5)) * p + 3.0 * tra_coef(4)) * p + ...
                2.0 * tra_coef(3)) * p + tra_coef(2);
        case 2 % accelerate ax^3 + bx^2 + cx + d = 0;
            value = (((20.0 * tra_coef(6) * p + 12.0 * tra_coef(5)) * p) + 6.0 * tra_coef(4)) * p +...
             2.0 * tra_coef(3);
        case 3 %jerk ax^2 + bx + c = 0;
            value = (60.0 * tra_coef(6) * p + 24.0 * tra_coef(5)) * p + 6.0 * tra_coef(4);
        case 4 % djerk ax + b 
            value = 120.0 * tra_coef(6) * p + 24.0 * tra_coef(5);
        case 5 %ddjerk
            value = 120.0 * tra_coef(6);
    end
end

%% brief calculate coefficients of quintic polynomial
%  s = f(t)
% x0 start point x0 position dx0 velocity ddx0 accelerate 
% x1 end point x1 position dx1 velocity ddx1 accelerate 
% time of from start point to end point
function Coef = QuinticPolynomialComputeCoefficients(x0, dx0, ddx0,  x1, dx1, ddx1,  p) 
    Coef(1) = x0;
    Coef(2) = dx0;
    Coef(3) = ddx0 / 2.0;
  
    p2 = p * p;
    p3 = p * p2;
  
    % the direct analytical method is at least 6 times faster than using matrix inversion.
    c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
    c1 = (dx1 - ddx0 * p - dx0) / p2;
    c2 = (ddx1 - ddx0) / p;

    Coef(4) = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    Coef(5) = (-15.0 * c0 + 7.0 * c1 - c2) / p;
    Coef(6) = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
end


%% calculate jerk max min and zero point t0 and t1
% Coef 五次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^5
% Tmin_ 五次多项式开始的时间 Tmax_ 五次多项式结束的时间
% JerkMax, JerkMin计算出最大最小的jerk
function [JerkMax, JerkMin] = CalcJerkRangeOfTrajectory(Coef, Tmax_, Tmin_)
    % jerk equation ax^2 + bx + c = 0; djerk equation ax + b = 0
    %最大的jerk可能在最大的端点
    jerk_1 = QuinticPolynomialEvaluate(Coef, 3, Tmin_);
    jerk_2 = QuinticPolynomialEvaluate(Coef, 3, Tmax_);
    JerkMax = max(jerk_1, jerk_2);
    JerkMin = min(jerk_1, jerk_2);
    %根据零点计算
    a = 120.0 * Coef(6);
    b = 24.0 * Coef(5);
    if b ~= 0
        x = -b/a;
        if x >= Tmin_ && x <= Tmax_
            jerk = QuinticPolynomialEvaluate(Coef, 3, x);
            JerkMax = max(JerkMax, jerk);
            JerkMin = min(JerkMin, jerk);
        end  
    end
end

%% calculate Acc max min and zero point t0 and t1
% Coef 五次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^6
% Tmin_ 五次多项式开始的时间 Tmax_ 五次多项式结束的时间
% AccMax, AccMin 计算出最大最小的加速度
function [AccMax, AccMin] = CalcAccRangeOfTrajectory(Coef, Tmax_, Tmin_)
    %计算两端的加速度值
    a0 = QuinticPolynomialEvaluate(Coef, 2, Tmin_);
    a1 = QuinticPolynomialEvaluate(Coef, 2, Tmax_);
    AccMax = max(a0, a1);
    AccMin = min(a0, a1);
    %计算零点的个数
    %jerk equation ax^2 + bx + c = 0;
    a = 60.0 * Coef(6);
    b = 24.0 * Coef(5);
    c = 6 * Coef(4);
    delta = b * b - 4 * a * c;
    if a == 0
        if b ~= 0
            %有一个零点
             x = -b/a;
            if x >= Tmin_ && x <= Tmax_
                acc = QuinticPolynomialEvaluate(Coef, 2, x);
                AccMax = max(AccMax, acc);
                AccMin = min(AccMin, acc);
            end
        end
    else
        if delta > 0
            %两个零点
            x1 = (-b - sqrt(delta))/(2 * a);
            if x1 >= Tmin_ && x1 <= Tmax_
                acc_1 = QuinticPolynomialEvaluate(Coef, 2, x1);
                AccMax = max(AccMax, acc_1);
                AccMin = min(AccMin, acc_1);
            end 
            x2 = (-b + sqrt(delta))/(2 * a);
            if x2 >= Tmin_ && x2 <= Tmax_
                acc_2 = QuinticPolynomialEvaluate(Coef, 2, x2);
                AccMax = max(AccMax, acc_2);
                AccMin = min(AccMin, acc_2);
            end 
        elseif delta == 0
                %一个零点
                 x = -b / (2*a);
                if x >= Tmin_ && x <= Tmax_
                    acc = QuinticPolynomialEvaluate(Coef, 2, x);
                    AccMax = max(AccMax, acc);
                    AccMin = min(AccMin, acc);
                end
        else
            %没有零点
        end
    end
end

%% calculate V max min
% Coef 五次多项式的系数 y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^6
% Tmin_ 五次多项式开始的时间 Tmax_ 五次多项式结束的时间
% VMax, VMin 计算出最大最小的加速度
function [VMax, VMin] = CalcVRangeOfTrajectory(Coef, Tmax_, Tmin_)
    %计算两端的加速度值
    VMax = 0;
    VMin = 0;
    for i = Tmin_:0.05:Tmax_
        t = i;
        v0 = QuinticPolynomialEvaluate(Coef, 1, t);
        VMax = max(v0, VMax);
        VMin = min(v0, VMin);
    end
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



%% calculate s_upper s_lower v_upper v_lower
%InitS(1) 位置 InitS(2)速度 InitS(3)加速度
%时间ti
function State = CalcStateOfTrajectory(InitS, Ti)
State = struct('s_upper',0, 's_lower',0 , 'v_upper', 0 , 'v_lower',0);
max_deceleration = -5;
max_acceleration = 2;
v_max = 30;%自车最大车速
%减速到0时的时间和位置
t_at_zero_speed = InitS(2) / max_deceleration;
s_at_zero_speed = InitS(1) + abs(InitS(2) * InitS(2) / (2.0 * max_deceleration));
% 加速到v_max时间
t_at_v_max = (v_max - InitS(2)) / max_acceleration;

if Ti <= t_at_v_max
    %匀加速 s0+ v0 * t + 0.5 * acc * t^2
    State.s_upper = InitS(1) + InitS(2) * Ti + 0.5 * max_acceleration * Ti^2;
    %v0 + at
    State.v_upper = InitS(2) + max_acceleration * Ti;
else
    %匀加速加匀速 s0+ v0 * t + 0.5 * acc * t^2 + v_max * (Ti - t_at_v_max)
    State.s_upper = InitS(1) + InitS(2) * t_at_v_max + 0.5 * max_acceleration * t_at_v_max^2 +  v_max * (Ti - t_at_v_max);
    State.v_upper = v_max;
end

if Ti < t_at_zero_speed
    %匀减速
    State.s_lower = InitS(1) + InitS(2) * Ti + 0.5 * max_deceleration * Ti^2;
    % v0 + at
    State.v_lower = InitS(2) + max_deceleration * Ti; 
else
    State.s_lower = s_at_zero_speed;
    State.v_lower = 0;
end

end


%% gcalculate target accelerate
function acc = GetTargetAcc(delta_s, delta_v)
    %delta_s  si - s_target 都是相对于前车的距离
    %delta_v vi - v_target
    sv_rear1_alpha = 0.05;
    sv_rear1_kesai = -0.12;

    sv_rear2_alpha = 0.01;
    sv_rear2_kesai = -0.06;

    sv_rear3_alpha = -0.05;
    sv_rear3_kesai = 0.1;

    if delta_s > 0 && delta_v > 0
        acc = max(-delta_v^2/(2 * delta_s)*0.4, -1.5);
    elseif delta_s <= 0 && delta_v >= 0
        acc = max(delta_s * sv_rear1_alpha + delta_v * sv_rear1_kesai, -1.5);
    elseif delta_s >= 0 && delta_v <= 0
        acc = delta_s * sv_rear2_alpha + delta_v * sv_rear2_kesai;
    else
        acc = max(min(delta_s * sv_rear3_alpha + delta_v * sv_rear3_kesai, 1.0), -1.5);
    end
end








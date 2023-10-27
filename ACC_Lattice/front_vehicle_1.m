%% 跟随前车

%% 约束限制和权重
JerkWeight = 10;

JerkLimitMax = 3;
JerkLimitMin = -5;
AccLimitMax = 2.0;
AccLimitMin = -5.5;

VLimitMax = 30;
VLimitMin = 12;

kJerkRangeY = [0 0 0.2 0.8 2];
kJerkRangeX = [0 1  2   3  4];

%%
tra_score = 100;
tra_jerk_min = 4;% default
tra_result = false;
tra_time = 30;
tra_coef = zeros(6,1);

%% 自车状态和目标速度
end_time = 15;%多长时间到大目标位置 目标速度
%自车状态
ego_s0 = 20;
ego_v = 68/3.6;
ego_a = 0.0;
%前车的状态
front_s1 = 100;
front_v = 0/3.6;
front_a = 0;
% end time 时间后
target_s = front_s1 + front_v * end_time - front_v * 1.5;
target_v = front_v;
target_a = 0;

%% 采样的方式
for ti = 0.01 : 0.5 : 10
    target_s = front_s1 + front_v * ti - front_v * 1.5;
    target_v = front_v;
    target_a = 0;
    [jerk_max, jerk_min, acc_max, acc_min, coef] = GetTrajectory(ego_s0, ego_v, ego_a,  target_s, target_v, target_a, ti);
    if jerk_max <= JerkLimitMax && jerk_min >= JerkLimitMin && ...
            acc_max <= AccLimitMax && acc_min >= AccLimitMin
            tra_jerk_min = max(jerk_max,abs(jerk_min));
            % calculate cost of trajectory include jerk
            score = interp1(kJerkRangeX, kJerkRangeY, tra_jerk_min) * JerkWeight;
            if score < tra_score
                tra_score = score;
                tra_result = true;
                tra_coef = coef;
                tra_time = ti;
            end
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
    for i = 1:tra_num
        time_arr(i) = i*0.1;
        position_arr(i) = Evaluate(tra_coef, 0, i*0.1);
        velocity_arr(i) = Evaluate(tra_coef, 1, i*0.1);
        accelerate_arr(i) = Evaluate(tra_coef, 2, i*0.1);
        jerk_arr(i) = Evaluate(tra_coef, 3, i*0.1);    
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




%%  calculate s = f(t)
function [JerkMax, JerkMin, AccMax, AccMin, Coef] = GetTrajectory(s0, v0, a0, st, vt, at, tend)
% start point 
%s0 = s0;
ds0 = v0; % velocity
dds0 = a0; % accelerate
% end point
s1 = st;
ds1 = vt;
dds1 = at;
% time 
all_time = tend;
% get tra_coef 5s
tra_coef = ComputeCoefficients(s0, ds0, dds0,  s1, ds1, dds1, all_time);
Coef = tra_coef;
%
t_min = 0;
t_max = all_time;
t0 = 0;% zero point 
t1 = 0;
[JerkMax, JerkMin, t0, t1] = CalculateJerkOfTrajectory(tra_coef, t_max, t_min);
[AccMax, AccMin] = CalculateAccOfTrajectory(tra_coef, t_max, t_min, t0, t1);
end


%% calculate jerk max min and zero point t0 and t1
function [JerkMax, JerkMin, T0, T1] = CalculateJerkOfTrajectory(tra_coef, Tmax_, Tmin_)
    T0 = 0;
    T1 = 0;
    %jerk equation ax^2 + bx + c = 0;
    a = 60.0 * tra_coef(6);
    b = 24.0 * tra_coef(5);
    c = 6 * tra_coef(4);
    delta = b * b - 4 * a * c;
    if delta < 0
              % no zero point
    elseif delta == 0
        x = -b / (2*a);
        if x >= Tmin_ && x <= Tmax_
            T0 = x; % get one zero point
        end
    else
        x1 = (-b - sqrt(delta))/(2 * a);
        if x1 >= Tmin_ && x1 <= Tmax_
            T0 = x1; % get one zero point
        end 
        x2 = (-b + sqrt(delta))/(2 * a);
        if x2 >= Tmin_ && x2 <= Tmax_
            T1 = x2; % get one zero point
        end 
    end
    % calculate jerk max and min
    % ax + b = 0
    jerk_1 = Evaluate(tra_coef, 3, Tmin_);
    jerk_2 = Evaluate(tra_coef, 3, Tmax_);
    JerkMax = max(jerk_1, jerk_2);
    JerkMin = min(jerk_1, jerk_2);
    a = 120.0 * tra_coef(6);
    b = 24.0 * tra_coef(5);
    if b ~= 0
        x = -b/a;
        if x >= Tmin_ && x <= Tmax_
            jerk = Evaluate(tra_coef, 3, x);
            JerkMax = max(JerkMax, jerk);
            JerkMin = min(JerkMin, jerk);
        end  
    end
end

%% calculate Acc max min 
function [AccMax, AccMin] = CalculateAccOfTrajectory(tra_coef, Tmax_, Tmin_, T0_, T1_)
    a0 = Evaluate(tra_coef, 2, Tmin_);
    a1 = Evaluate(tra_coef, 2, Tmax_);
    AccMax = max(a0, a1);
    AccMin = min(a0, a1);
    if T0_ > Tmin_ && T0_ <= Tmax_
        a = Evaluate(tra_coef, 2, T0_);
        AccMax = max(AccMax, a);
        AccMin = min(AccMin, a);
    end
    if T1_ > Tmin_ && T1_ <= Tmax_
        a = Evaluate(tra_coef, 2, T1_);
        AccMax = max(AccMax, a);
        AccMin = min(AccMin, a);
    end
end
%% get value from s = f(t) quintic polynomial
% order derivation
% p time of all
function value = Evaluate(tra_coef, order, p)
    value = 0;
    switch (order) 
        case 0 % position
            value = ((((tra_coef(6) * p + tra_coef(5)) * p + tra_coef(4)) * p + tra_coef(3)) * p + ...
                tra_coef(2)) * p + tra_coef(1);
        case 1 % velocity
            value = (((5.0 * tra_coef(6) * p + 4.0 * tra_coef(5)) * p + 3.0 * tra_coef(4)) * p + ...
                2.0 * tra_coef(3)) * p + tra_coef(2);
        case 2 % accelerate
            value = (((20.0 * tra_coef(6) * p + 12.0 * tra_coef(5)) * p) + 6.0 * tra_coef(4)) * p +...
             2.0 * tra_coef(3);
        case 3 %jerk
            value = (60.0 * tra_coef(6) * p + 24.0 * tra_coef(5)) * p + 6.0 * tra_coef(4);
        case 4 % djerk
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
function coef = ComputeCoefficients(x0, dx0, ddx0,  x1, dx1, ddx1,  p) 
    coef(1) = x0;
    coef(2) = dx0;
    coef(3) = ddx0 / 2.0;
  
    p2 = p * p;
    p3 = p * p2;
  
    % the direct analytical method is at least 6 times faster than using matrix inversion.
    c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
    c1 = (dx1 - ddx0 * p - dx0) / p2;
    c2 = (ddx1 - ddx0) / p;
  
    coef(4) = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    coef(5) = (-15.0 * c0 + 7.0 * c1 - c2) / p;
    coef(6) = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
end












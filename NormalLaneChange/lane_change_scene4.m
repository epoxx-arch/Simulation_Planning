
%% scenery olny side rear 1 
    % 只有后车 和前车
JerkWeight = 60;
VMaxWeight = 20;
TimeWeight = 100 - JerkWeight;
JerkLimitRange = 4;
TimeRange = 8;
JerkLimitMax = 3;
JerkLimitMin = -4;
AccLimitMax = 2.4;
AccLimitMin = -3.5;
SearchSOffset = 6; %6米
VLimitMax = 30;
VLimitMin = 12;
CrashDeceToFront = 0.4;% -0.6m/s
CrashDeceToRear = 0.6;% -0.6m/s
kJerkRangeY = [0 0 0.2 0.8 2];
kJerkRangeX = [0 1  2   3  4];
kTimeRangeY = [0  0  0.4 1.0 1.4  2.2];
kTimeRangeX = [3  4   5   6   7    8];
kVMaxRangeY = [0  0.4   0.8  1.2   1.6];
kVMaxRangeX = [0   5    10    20  30];
kEgoLength = 5;
%% surrounding vehicle information
velocity_side_rear = 23;
position_side_rear = -20;
% ego
ego_pos = 0;
ego_v = 25;
ego_a = 0;
%% top trajectory infomation
tra_score = 100;
tra_jerk_min = 4;% default
tra_result = false;
tra_postion = 0;
tra_velocity = 0;
tra_direction = 0;
tra_time = 30;
tra_coef = zeros(6,1);
%%
for ti = 3 : 8 % time i is 3 4 5 6 7 8s
    %prediction position after ti s
    prediction_position_side_rear = velocity_side_rear * ti + position_side_rear;
    safe_side_rear = prediction_position_side_rear + velocity_side_rear * 0.8;
    % ego accelerate from vego to VLimitMax and acc is 1m/s^2
    % calculate time of the accelerate 
    s_end = 0;
    v_end = 0;
    t0 = (VLimitMax - ego_v)/1.5; 
    if t0 >= ti
        % velocity of ego not add to VLimitMax 
        s_end = ego_v * ti + 0.5 * 1.5 * ti^2;
        v_end = ego_v + 1.5 * ti;
    else
        s_end = ego_v * t0 + 0.5 * 1.5 * t0^2 + VLimitMax * (ti - t0);
        v_end = VLimitMax;
    end
    % S gap of the window
    window_gap = s_end - safe_side_rear;
    if window_gap > kEgoLength %
        window_gap_start = safe_side_rear;
        window_gap_end = s_end;
        si = s_end;
        vi = v_end;
        while si > window_gap_start
            v_max = v_end;
            v_min = min(velocity_side_rear - sqrt(2*(si - prediction_position_side_rear)*CrashDeceToRear),...
                VLimitMin);
            if v_max > v_min + 1
                st = si;
                vt = v_min;
                at = 0;
                while vt < v_max
                    [jerk_max, jerk_min, acc_max, acc_min, coef] = GetTrajectory(ego_pos, ego_v, ego_a, st, vt, at, ti);
                    if jerk_max <= JerkLimitMax && jerk_min >= JerkLimitMin && ...
                            acc_max <= AccLimitMax && acc_min >= AccLimitMin
                        tra_jerk_min = max(jerk_max,abs(jerk_min));
                        score = interp1(kJerkRangeX, kJerkRangeY, tra_jerk_min) * JerkWeight ...
                                + interp1(kTimeRangeX, kTimeRangeY, ti) * TimeWeight ...
                                + interp1(kVMaxRangeX, kVMaxRangeY, (VLimitMax - vt)) * VMaxWeight;
                        if score < tra_score
                            tra_score = score;
                            tra_result = true;
                            tra_postion = si;
                            tra_velocity = vt;
                            tra_direction = 1;
                            tra_time = ti;
                            tra_coef = coef;
                        end
                    end
                    vt = vt + 1;% v into next layer
                end
            end
            si = si - 6;% s into next layer
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
    tra_num = tra_time/0.1; % 0.1 get one point
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

%% replay
if tra_result == true
    %定义道路宽度  
    LaneWidth = 4;%
    LaneLength = 800;%路长800米
    global_path_x = 0: 0.2 : LaneLength;%
    num1 = uint16(length(global_path_x));%计算多少个点
    global_path_y = zeros(1,num1);
    leftLaneMarker = global_path_y + LaneWidth;%左车道线的y坐标值
    rightLaneMarker = global_path_y - LaneWidth; 
    leftleftLaneMarker = leftLaneMarker + LaneWidth;
    rightrightLaneMarker = rightLaneMarker - LaneWidth;
    %%
    %显示自车位置   %%绘制轨迹点
    %plot(global_path_x,global_path_y,'b--',  global_path_x,leftLaneMarker,'k-', global_path_x,rightLaneMarker,'k-');
    figure(5);
    for i = 1 : tra_num
        plot(global_path_x,global_path_y,'b--',  global_path_x,leftLaneMarker,'k-', global_path_x,rightLaneMarker,'k-');
        EgoWidth = 2;%车宽
        EgoLength = 5;
        EgoPosX = position_arr(i);
        EgoPosY = 0;
        EgoHeading = 0;
        EgoColor = [0.9 0.9 0.9 0.9];
        Boxyx = zeros(1,4);
        Boxyy = zeros(1,4);
        Boxyx(1) = EgoPosX - EgoWidth/2*sin(EgoHeading);
        Boxyy(1) = EgoPosY + EgoWidth/2*cos(EgoHeading);
        Boxyx(2) = EgoPosX + EgoWidth/2*sin(EgoHeading);
        Boxyy(2) = EgoPosY - EgoWidth/2*cos(EgoHeading);
        Boxyx(3) = Boxyx(2) + EgoLength*cos(EgoHeading);
        Boxyy(3) = Boxyy(2) + EgoLength*sin(EgoHeading);
        Boxyx(4) = Boxyx(1) + EgoLength*cos(EgoHeading);
        Boxyy(4) = Boxyy(1) + EgoLength*sin(EgoHeading);
        patch(Boxyx,Boxyy,EgoColor);
        axis([EgoPosX-40,EgoPosX+65,-30,30]);%坐标范围
        %变道窗口的后车
        objWidth = 2;
        objLength = 5;
        objPosX = position_side_rear + velocity_side_rear * 0.1 * i;
        objPosY = -4;
        objHeading = 0;
        objColor = [0.9 0.9 0.9 0.9];
        Boxyx(1) = objPosX - objWidth/2*sin(objHeading);
        Boxyy(1) = objPosY + objWidth/2*cos(objHeading);
        Boxyx(2) = objPosX + objWidth/2*sin(objHeading);
        Boxyy(2) = objPosY - objWidth/2*cos(objHeading);
        Boxyx(3) = Boxyx(2) + objLength*cos(objHeading);
        Boxyy(3) = Boxyy(2) + objLength*sin(objHeading);
        Boxyx(4) = Boxyx(1) + objLength*cos(objHeading);
        Boxyy(4) = Boxyy(1) + objLength*sin(objHeading);
        patch(Boxyx,Boxyy,objColor);
        pause(0.2);
    end
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



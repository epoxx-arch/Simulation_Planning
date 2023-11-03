%% 两段jerk预测
%根据滤波之后的加速度预测前车的车速
%输入 前车的位置 速度 加速度 输出前车ti时刻的状态 ti的范围0-16秒
%预测前车分三个阶段 1、jerk = a 2、jerk = 0 3、jerk = -a  4、jerk = 0 且 a = 0;
front_s = 0;%w位置
front_v = 5;%速度
front_a = -2;%匀减速
front_jerk = 0;%当前估计的前车jerk值
ti = 5;
if abs(front_a) <= 0.3
    %预测前车匀速直线运动
    %ti时刻的状态
    st = front_s + front_v * ti;
    vt = front_v;
    at = 0;
else
    %前车减速运动
    if front_a < 0
        %情况1 front_a 小于0 front_jerk为正
        if front_jerk > 0
            t1 = (0 - front_a)/front_jerk;%
            jerk1 = front_jerk;
            %考虑计算front_jerk很小，front_a比较大，t1会比较大。
            %因此强制t1最大6秒
            if t1 > 6
                jerk1 = (0 - front_a)/6;
                t1 = 6;
            end
            %情况1下，前车的速度预测分两段
            if ti <= t1
                %ti时刻的状态
                st = front_s + front_v * ti + 0.5 * front_a * ti^2 + 1/6 * jerk1 * ti^3;
                vt = min(front_v + front_a * ti + 0.5 * jerk1 * ti^2, 0);%还需要考虑车速可能是负的情况
                at = front_a + jerk1 * ti;
            else
                %当 ti > t1 匀jerk + 匀速
                vt = min(front_v + front_a * t1 + 0.5 * jerk1 * t1^2, 0);%还需要考虑车速可能是负的情况
                st = front_s + front_v * t1 + 0.5 * front_a * t1^2 + 1/6 * jerk1 * t1^3  + vt * (ti - t1);
                at = 0;
            end
        else
            %情况2 front_a 小于0 front_jerk为负
            %分三段 1、前车jerk负  2、jerk = 0 3、jerk 正 4、匀速
            %考虑预测的某一段速度为0
            t1 = 2;%假设的2秒 ？？
            jerk1 = front_jerk;
            a1 = front_a + jerk1 * t1;%第一段结束时的加速度
            v1 = front_v + front_a * t1 + 0.5 * jerk1 * t1^2; % t1结束时的速度
            
            t2 = 1;%假设 ？？
            jerk2 = 0;
            a2 = a1;
            v2 = v1 + a2 * t2;
            
            jerk3 = 1.6;%比较舒适的减速
            t3 = (0 - a2)/jerk3;
            a3 = 0;
            v3 = v2 + a2 * t3 + 0.5 * jerk3 * t3^2;
            
        end  
    else
        
    end
end

%% 两段预测 1、匀加速 2、舒适的jerk运动 3、匀速
front_s = 0;%w位置
front_v = 5;%速度
front_a = -2;%匀减速
front_jerk = 0;%当前估计的前车jerk值
ti = 5;
if abs(front_a) <= 0.3
    %预测前车匀速直线运动
    %ti时刻的状态
    st = front_s + front_v * ti;
    vt = front_v;
    at = 0;
else
    if front_a > 0
        %前车车速大于0 直接两段 jerk = 0 jerk 负
        %假设第一段jerk = 0 时间 t1 = 2秒 
        t1 = 2;
        jerk1 = 0;
        a1 = front_a;
        v1 = front_v + front_a * t1;
        s1 = front_s + front_v * t1 + 0.5 * front_a * t1^2;
        %假设第二段 jerk = -1.6 舒适到匀速
        jerk2 = -1.6;
        t2 = abs((front_a - 0)/jerk2);
        a2 = 0;
        v2 = v1 + front_a * t2 + 0.5 * jerk2 * t2^2;
        s2 = s1 + v1 * t2 + 0.5 * a1 * t2^2 + 1/6 * jerk2 * t2^3;
        %计算ti时候前车的状态
        if ti <= t1
            st = front_s + front_v * ti + 0.5 * front_a * ti^2;
            vt = front_v + front_a * ti;
            at = front_a; 
        elseif ti <= (t1 + t2)
            %加速度 -> 0kph
            t = (ti - t1);
            st = s1 + v1 * t + 0.5 * a1 * t^2 + 1/6 * jerk2 * t^3;
            vt = v1 + a1 * t + 0.5 * jerk2 * t^2;
            at = a1 + jerk2 * t; 
        else
            % ti > (t1 + t2) 匀速
            t = ti - (t1 + t2);
            st = s2 + v2 * t;
            vt = v2;
            at = 0;
        end
    else
        % front_a < 0
        %假设前车舒适的减速 front_a -> 0 jerk = 1.6
        t1 = 0; jerk1 = 0; a1 = 0; v1 = 0;
        t2 = 0; jerk2 = 0; a2 = 0; v2 = 0;
        
        jerk = 1.6;
        t_end = (0 - front_a)/jerk;
        v_end = front_v + front_a * t_end + 0.5 * jerk * t_end^2;
        if v_end < 0
            %一段jerk = 1.6 然后一段匀速
            a = 0.5 * jerk;
            b = front_a;
            c = front_v ;
            [res, val] = CalcLinearEquation(a, b, c, 0, t_end);
            if (res)
                t1 = val; 
                jerk1 = 1.6; 
                a1 = 0; 
                v1 = 0;
                s1 = front_s + front_v * t1 + 0.5 * front_a * t1^2 + 1/6 * jerk1 * t1^3;
                
                t2 = 0; 
                jerk2 = 0; 
                a2 = 0; 
                v2 = 0;
            else
                %无解 就直接匀减速车速到0
                t1 = (0 - front_a)/front_v; 
                jerk1 = 0; 
                a1 = 0; 
                v1 = 0;
                t2 = 0; 
                jerk2 = 0; 
                a2 = 0; 
                v2 = 0; 
            end
        else
            %v_end > 0
            
        end
    end
    
end


function [res, val] = CalcLinearEquation(a, b, c, min, max)
res = 0;
val = 0;
delta = b^2 - 4 * a * c;
if delta < 0
    %无解
    res = 0;
elseif delta == 0
    %一个接 
    x = -b/2;
    if x >= max && x <= min
        val = x;
        res = 1;
    end
else
    x_1 = (-b + sqrt(delta))/(2 * a);
    if x_1 >= max && x_1 <= min
        val = x_1;
        res = 1;
    else
        x_2 = (-b - sqrt(delta))/(2 * a);
        if x_2 >= max && x_2 <= min
            val = x_2;
            res = 1;
        end
    end
    
end

end





























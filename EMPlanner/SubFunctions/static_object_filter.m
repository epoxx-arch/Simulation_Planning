function [consider_static_objects_set] = static_object_filter(...
    ego, static_objects_set, ...
    plan_referenceline_x, plan_referenceline_y, plan_referenceline_heading, plan_referenceline_kappa,...
    plan_referenceline_s)
static_object_num = 10;
[filter_static_objects] = static_objects_filter(ego, static_objects_set);
% 需要规划处理的障碍物且计算障碍物的四个角点
% consider_static_object_num = 10 最多处理10个障碍物
consider_static_object_num = length(filter_static_objects);
consider_static_object = struct('id',0,'decision',0,'site',0,'x_set',zeros(4,1),'y_set',zeros(4,1),...
    's_set',zeros(4,1),'l_set',zeros(4,1),'min_s',inf,'max_s',-inf,'min_l',inf,'max_l',-inf);
consider_static_objects_set = repmat(consider_static_object,static_object_num,1);%静态障碍物默认10个
for i = 1 : static_object_num
    if filter_static_objects(i).valid == 0 %invalid
        continue;
    end
    consider_static_objects_set(i).id = i;
    objPosX = filter_static_objects(i).x;
    objPosY = filter_static_objects(i).y;
    objHeading = filter_static_objects(i).heading;
    objWidth = filter_static_objects(i).width;
    objLength = filter_static_objects(i).length;
    consider_static_objects_set(i).x_set(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 

    consider_static_objects_set(i).x_set(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);

    consider_static_objects_set(i).x_set(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);

    consider_static_objects_set(i).x_set(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    consider_static_objects_set(i).y_set(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
end

% 计算考虑静态的障碍物在frenet下面的sl
for i = 1 : static_object_num
    if consider_static_objects_set(i).id == 0
        continue;
    end
    %静态障碍物的四个角点投影到 plan_referenceline 上
    for j = 1 : length(consider_static_objects_set(i).x_set)
        static_obj_x = consider_static_objects_set(i).x_set(j);
        static_obj_y = consider_static_objects_set(i).y_set(j);
        [match_point_index, proj_x, proj_y, proj_heading, proj_kappa] = matchpoint(...
        static_obj_x ,static_obj_y ,...
        plan_referenceline_x, plan_referenceline_y, plan_referenceline_heading, plan_referenceline_kappa);
        [static_obj_s,static_obj_l] = Cartesian2Frenet_sl(static_obj_x, static_obj_y,...
            plan_referenceline_x,plan_referenceline_y,...
            proj_x,proj_y,proj_heading,match_point_index,plan_referenceline_s);
        consider_static_objects_set(i).s_set(j) = static_obj_s;
        consider_static_objects_set(i).l_set(j) = static_obj_l;
        if consider_static_objects_set(i).min_s > static_obj_s
            consider_static_objects_set(i).min_s = static_obj_s;
        end
        if consider_static_objects_set(i).max_s < static_obj_s
            consider_static_objects_set(i).max_s = static_obj_s;
        end
        if consider_static_objects_set(i).min_l > static_obj_l
            consider_static_objects_set(i).min_l = static_obj_l;
        end
        if consider_static_objects_set(i).max_l < static_obj_l
            consider_static_objects_set(i).max_l = static_obj_l;
        end
    end
end

% 添加对静态障碍物的决策
for i = 1 : static_object_num
    if consider_static_objects_set(i).id == 0
        continue;
    end
    if consider_static_objects_set(i).min_l < 0 && consider_static_objects_set(i).max_l < 0
        %障碍物在path_reference右边 1 is site = right 
        consider_static_objects_set(i).site = 1;
        if consider_static_objects_set(i).max_l <= -0.5
            consider_static_objects_set(i).decision = 1;% decision = 1 nudge
        else
            consider_static_objects_set(i).decision = 2;% decision = 2 stop
        end
    end
    if consider_static_objects_set(i).min_l > 0 && consider_static_objects_set(i).max_l > 0
        %障碍物在path_reference左边 2 is site = left 
        consider_static_objects_set(i).site = 2;
        if consider_static_objects_set(i).max_l > 0.8
            consider_static_objects_set(i).decision = 1;% decision = 1 nudge
        else
            consider_static_objects_set(i).decision = 2;
        end
    end
    if consider_static_objects_set(i).min_l < 0 && consider_static_objects_set(i).max_l > 0
        %障碍物在path_reference中间 直接stop决策 3 is site = right
        consider_static_objects_set(i).site = 3;
        consider_static_objects_set(i).decision = 2;
    end
end 

end
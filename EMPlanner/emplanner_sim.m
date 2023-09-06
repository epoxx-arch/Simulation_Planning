%% 此文件用来测试单一path trajectory 规划

%% 计算投影点
%根据自车的位置在全局的referenceline上面找到自车的投影点
[ego_match_point_index,ego_proj_x, ego_proj_y,ego_proj_heading,ego_proj_kappa] = matchpoint(...
    ego.x ,ego.y ,...
    lane_1_reference_line_x,lane_1_reference_line_y,line_1_reference_line_heading, lane_1_reference_line_kappa);
%在全局的referenceline上截取一段作为规划的路径参考线plan_referenceline
[plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading,plan_referenceline_kappa] = get_plan_referenceline(...
ego_match_point_index,lane_1_reference_line_x,lane_1_reference_line_y,...
line_1_reference_line_heading , lane_1_reference_line_kappa);
%自车的位置x y投影到路径参考线plan_referenceline获得匹配点和投影点坐标朝向曲率
[ego_match_point_index,ego_proj_x, ego_proj_y,ego_proj_heading,ego_proj_kappa] = matchpoint(...
    ego.x ,ego.y ,...
    plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading, plan_referenceline_kappa);
%计算plan_referenceline_s上面的累积 s点
plan_referenceline_s  = get_referenceline_s(plan_referenceline_x,plan_referenceline_y,ego_proj_x,ego_proj_y,ego_match_point_index);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 s l
[ego_plan_start_s,ego_plan_start_l] = Cartesian2Frenet_sl(ego.x,ego.y,plan_referenceline_x,plan_referenceline_y,...
    ego_proj_x,ego_proj_y,ego_proj_heading,ego_match_point_index,plan_referenceline_s);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 dl
[ego_plan_start_s_dot,ego_plan_start_l_dot,ego_plan_start_dl] = Cartesian2Frenet_dsl(ego_plan_start_l,ego.vx,ego.vy,...
    ego_proj_heading,ego_proj_kappa);
%自车规划的起点Cartesian2Frenet_sl 获得规划起点的 ddl
[~,~,ego_plan_start_ddl] = Cartesian2Frenet_ddsl(ego.ax,ego.ay,...
    ego_proj_heading,ego_proj_kappa,ego_plan_start_l,ego_plan_start_s_dot,ego_plan_start_dl);
%规划起点的frenet
plan_start_s = ego_plan_start_s;
plan_start_l = ego_plan_start_l;
plan_start_dl = ego_plan_start_dl;
plan_start_ddl = ego_plan_start_ddl;
%% 静态障碍物的处理
[consider_static_objects_set] = static_object_filter(...
    ego, static_objects_set, ...
    plan_referenceline_x, plan_referenceline_y, plan_referenceline_heading, plan_referenceline_kappa,...
    plan_referenceline_s);

%% 路径的动态规划获得凸空间
[l_min, l_max, dp_path_s] = generate_convex_space_path(consider_static_objects_set, plan_start_s);

%% 路径二次规划
% 0.5*x'Hx + f'*x = min
% subject to A*x <= b
%            Aeq*x = beq
%            lb <= x <= ub;
% 输入：l_min l_max 点的凸空间
[qp_path_s, qp_path_l, qp_path_dl, qp_path_ddl] = qp_path(...
    ego_plan_start_s, ego_plan_start_l, ego_plan_start_dl, ego_plan_start_ddl,...
    l_max, l_min);
% frenet坐标系转到世界坐标系 获得初始的路径
[trajectory_x_init, trajectory_y_init, trajectory_heading_init, trajectory_kappa_init] = Frenet2Cartesian_Path(...  
 qp_path_s,qp_path_l, qp_path_dl,qp_path_ddl,...
 plan_referenceline_x,plan_referenceline_y,plan_referenceline_heading, plan_referenceline_kappa,plan_referenceline_s);

%% 动态障碍物



%% 显示车道线 自车 障碍物
%显示规划路径参考线plan_referenceline
figure(3);
% map + 规划的轨迹点
plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-',...
trajectory_x_init, trajectory_y_init,'b.');

% ego
objWidth = ego.width;
objLength = ego.length;
objPosX = ego.x;
objPosY = ego.y;
objHeading = ego.heading;
Boxyx(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
Boxyy(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
Boxyx(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 
Boxyx(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
Boxyy(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);
patch(Boxyx,Boxyy,'blue'); 
%静态障碍物
for j = 1:length(static_objects_set)
    objWidth = static_objects_set(j).width;
    objLength = static_objects_set(j).length;
    objPosX = static_objects_set(j).x;
    objPosY = static_objects_set(j).y;
    objHeading = static_objects_set(j).heading;
    Boxyx(3) = objPosX - objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    Boxyy(3) = objPosY + objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
    Boxyx(4) = objPosX + objWidth/2*sin(objHeading) + objLength/2*cos(objHeading);
    Boxyy(4) = objPosY - objWidth/2*cos(objHeading) + objLength/2*sin(objHeading);
    Boxyx(1) = objPosX + objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    Boxyy(1) = objPosY - objWidth/2*cos(objHeading) - objLength/2*sin(objHeading); 
    Boxyx(2) = objPosX - objWidth/2*sin(objHeading) - objLength/2*cos(objHeading);
    Boxyy(2) = objPosY + objWidth/2*cos(objHeading) - objLength/2*sin(objHeading);    
    patch(Boxyx,Boxyy,'black');
end  
axis([ego.x-10,ego.x + 80, -30, 30]);%根据自车定义范围坐标范围


















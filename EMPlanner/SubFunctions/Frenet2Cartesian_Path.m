function [x_set,y_set,heading_set,kappa_set] = ...
    Frenet2Cartesian_Path(s_set,l_set,dl_set,ddl_set,frenet_path_x,frenet_path_y,frenet_path_heading,frenet_path_kappa,index2s)
% 通用算法 frenet转frenet
% 由于不知道有多少个(s,l)要转化成直角坐标，因此做缓冲
% 输出初始化
    num = length(s_set);
    x_set = ones(num,1) * nan;
    y_set = ones(num,1) * nan;
    heading_set = ones(num,1) * nan;
    kappa_set = ones(num,1) * nan;
    for i = 1:length(s_set)
        if isnan(s_set(i))
            break;
        end
        % 计算(s,l)在frenet坐标轴上的投影
        [proj_x,proj_y,proj_heading,proj_kappa] = CalcProjPoint(s_set(i),frenet_path_x,frenet_path_y,frenet_path_heading,...
    frenet_path_kappa,index2s);
        nor = [-sin(proj_heading);cos(proj_heading)];
        point = [proj_x;proj_y] + l_set(i) * nor;
        x_set(i) = point(1);
        y_set(i) = point(2);
        heading_set(i) = proj_heading + atan(dl_set(i)/(1 - proj_kappa * l_set(i)));
        % 近似认为 kappa' == 0,frenet转cartesian公式，见第一章第三节评论区的链接
        kappa_set(i) = ((ddl_set(i) + proj_kappa * dl_set(i) * tan(heading_set(i) - proj_heading)) * ...
            (cos(heading_set(i) - proj_heading)^2)/(1 - proj_kappa * l_set(i)) + proj_kappa) * ...
            cos(heading_set(i) - proj_heading)/(1 - proj_kappa * l_set(i));
         
    end

end


function [proj_x,proj_y,proj_heading,proj_kappa] = CalcProjPoint(s,frenet_path_x,frenet_path_y,frenet_path_heading,...
    frenet_path_kappa,index2s)
% 该函数将计算在frenet坐标系下，点(s,l)在frenet坐标轴的投影的直角坐标(proj_x,proj_y,proj_heading,proj_kappa)'
% 先找匹配点的编号
    match_index = 1;
    while index2s(match_index) < s
        match_index = match_index + 1;
    end
    match_point = [frenet_path_x(match_index);frenet_path_y(match_index)];
    match_point_heading = frenet_path_heading(match_index);
    match_point_kappa = frenet_path_kappa(match_index);
    ds = s - index2s(match_index);
    match_tor = [cos(match_point_heading);sin(match_point_heading)];
    proj_point = match_point + ds * match_tor;
    proj_heading = match_point_heading + ds * match_point_kappa;
    proj_kappa = match_point_kappa;
    proj_x = proj_point(1);
    proj_y = proj_point(2);   
end

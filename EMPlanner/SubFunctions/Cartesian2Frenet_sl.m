function [s_set,l_set] = Cartesian2Frenet_sl(x_set,y_set,frenet_path_x,frenet_path_y,...
    proj_x_set,proj_y_set,proj_heading_set,proj_match_point_index_set,index2s)
    %该函数将计算世界坐标系下的x_set，y_set上的点在frenet_path下的坐标s l
    %输入 x_set,y_set 待坐标转换的点
    %     frenet_path_x,frenet_path_y   frenet坐标轴
    %     proj_x,y,heading,kappa,proj_match_point_index 待坐标转换的点的投影点的信息
    %     index2s   frenet_path的index与s的转换表

    % 由于不知道有多少个点需要做坐标转换，所以需要做缓冲
    n = length(x_set);%最多处理128个点
    %输出初始化
    s_set = ones(n,1)*nan;
    l_set = ones(n,1)*nan;
    for i = 1:length(x_set)
        if isnan(x_set(i))
            break;
        end
        %计算s，写个子函数
        s_set(i) = CalcSFromIndex2S(index2s,frenet_path_x,frenet_path_y,proj_x_set(i),proj_y_set(i),...
            proj_match_point_index_set(i));
        n_r = [-sin(proj_heading_set(i));cos(proj_heading_set(i))];
        r_h = [x_set(i);y_set(i)];
        r_r = [proj_x_set(i);proj_y_set(i)];
        l_set(i) = (r_h - r_r)'*n_r;
    end
end

function s = CalcSFromIndex2S(index2s,path_x,path_y,proj_x,proj_y,proj_match_point_index)
  %该函数将计算当指定index2s的映射关系后，计算点proj_x,proj_y的弧长
  vector_1 = [proj_x;proj_y] - [path_x(proj_match_point_index);path_y(proj_match_point_index)];
  %这里要考虑的更全面一些，因为要考虑到proj_match_point_index有可能在path的起点或终点
  if (proj_match_point_index < length(path_x))
      vector_2 = [path_x(proj_match_point_index + 1);path_y(proj_match_point_index + 1)] - ...
          [path_x(proj_match_point_index);path_y(proj_match_point_index)];
  else
      vector_2 = [path_x(proj_match_point_index);path_y(proj_match_point_index)] - ...
          [path_x(proj_match_point_index - 1);path_y(proj_match_point_index - 1)];
  end
  
  if vector_1'*vector_2 > 0
      s = index2s(proj_match_point_index) + sqrt(vector_1'*vector_1);
  else
      s = index2s(proj_match_point_index) - sqrt(vector_1'*vector_1);
  end
end










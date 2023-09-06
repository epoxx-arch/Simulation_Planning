
for i = 1:length(x_set - 10)
    plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
    ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
    ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-',...
    x_set,y_set,'b.',...
    dp_tra_s,dp_path_l_min,'r.',dp_tra_s,dp_path_l_max,'r.');
    %ego
    objWidth = ego.width;
    objLength = ego.length;
    objPosX = x_set(i);
    objPosY = y_set(i);
    objHeading = heading_set(i);
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
    axis([x_set(i)-10, x_set(i) + 30, -8, 8]);%根据自车定义范围坐标范围
    pause(0.2);
end
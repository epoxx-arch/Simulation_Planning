
for i = 1:length(trajectory_x_init - 10)
    plot(ReferenceLine_1_X,ReferenceLine_1_Y,'b--',  ReferenceLine_1_X,RL1_Left_LaneMarker,'k-',...
    ReferenceLine_1_X,RL1_Right_LaneMarker,'k-',...
    ReferenceLine_1_X,ReferenceLine_2_Y,'b--',ReferenceLine_1_X,RL2_Left_LaneMarker,'k-',...
    trajectory_x_init,trajectory_y_init,'b.');
    %ego
    objWidth = ego.width;
    objLength = ego.length;
    objPosX = trajectory_x_init(i);
    objPosY = trajectory_y_init(i);
    objHeading = trajectory_heading_init(i);
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
    axis([-10, 80, -30, 30]);%根据自车定义范围坐标范围
    pause(0.2);
end
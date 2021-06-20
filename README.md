# imu_utm_tf
该功能包是将组合惯导的经纬度信息、俯仰角、侧倾角、偏航角信息转换成ｉｍｕ相对与世界坐标系的tf转换关系发送出来。在这里使用的世界坐标系是ＵＴＭ平面投影坐标系，所以中间有从经纬度转换为ＵＴＭ坐标的步骤。
input: gps Lattitude,gps Longitude,pitch,yaw,heading
output:  tf::StampedTransform(transform, ros::Time::now(),"world","gnss")(旋转平移变换）


使用方法：
１．根据自己的imu消息格式修改ｍｓｇ文件，生成相应的头文件
２．catkin_make
3.source devel/setup.bash
4.roscore
5.rosrun 包名　节点名

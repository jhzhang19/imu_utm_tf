#include "ros/ros.h"
#include "imu_listener1/imu.h"
#include "../../include/imu_listener.h"
#include <tf/transform_broadcaster.h>
#include <vector>
//回调函数定义，参数类型需要与所订阅的消息类型保持一致
std::vector<float_t> longandlati_to_UTM(const float_t &longti, const float_t &lati )
{
  int zonenum = int(longti/6)+31;
  int lam_tem = (zonenum-1)*6-180+3;
  float_t lamda0 = float_t(lam_tem*M_PI/180); 
  float_t phi = lati*M_PI/180;
  float_t lamda = longti*M_PI/180;
  float_t v=0,A=0,T=0,C=0,s=0, UTME=0,UTMN = 0;
  v=float_t (1/sqrt(1-pow(e,2)*pow(sin(phi),2)));
  A = float_t((lamda - lamda0)*cos(phi));
  T = float_t (pow(tan(phi),2));
  C = float_t(pow(e,2)*pow(cos(phi),2)/(1-pow(e,2)));
  s = float_t(phi*(1-pow(e,2)/4-3*pow(e,4)/64-5*pow(e,6)/256)-sin(2*phi)*(pow(e,2)*3/8+pow(e,4)*3/32+pow(e,6)*45/1024)+sin(4*phi)*(pow(e,4)*15/256+pow(e,6)*45/1024)-sin(6*phi)*pow(e,6)*35/3072);
  UTME = (E0 +k0*a*v*(A+(1-T+C)*pow(A,3)/6+(5-18*T+pow(T,2))*pow(A,5)/120))*1000;
  UTMN = (N0 + k0*a*(s+v*tan(phi)*(pow(A,2)/2+(5-T+9*C+4*pow(C,2))*pow(A,4)/24+(61-58*T+pow(T,2))*pow(A,6)/720)))*1000;
  std::vector<float_t> result;
  result.resize(2,0);
  result[0] = UTME;
  result[1] = UTMN;
  return result;

}
void chatterCallback(const imu_listener1::imu::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f %f %f]", msg->Heading,msg->Pitch,msg->Roll);

  float_t utm_z = msg->Altitude;
  std::vector<float_t> result;
  result.resize(2,0);
  result=longandlati_to_UTM(msg->Longitude,msg->Lattitude);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(result[0],result[1],utm_z));
  tf::Quaternion q;
  q.setRPY(msg->Roll,msg->Pitch,msg->Heading);
  transform.setRotation(q);
  //广播world与imu之间的tf
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","gnss"));


 

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener"); //初始化ros，并命令节点名。
  ros::NodeHandle n; //初始化节点，调用ros api接口句柄。

  ros::Subscriber sub = n.subscribe("/imu_info", 200, chatterCallback); //定义一个订阅者。
 
  ros::spin(); //不返回函数，监听订阅者中的回调队列，并执行回调函数。
  /*
  float_t a=100;
  float_t b = 122;
  float_t c, d;
  longandlati_to_UTM(a,b,c,d);
  std::cout<<c<<" "<<d<<std::endl;
  */
 return 0; //正常运行时，不会执行到。
}
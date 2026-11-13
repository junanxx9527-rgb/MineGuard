// Gazebo simultaneous-snapshot LiDAR adapter, NOT a rotating time-resolved model.
// All points have the same acquisition time. A 1us positive offset avoids FAST-LIO
// interpreting missing/zero times as rotating-scan azimuth timing. Header+time
// equals the actual Gazebo sensor time; no ground-truth poses are read here.
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
namespace gazebo {
class MineGuardLioLidar : public SensorPlugin {
 sensors::RaySensorPtr sensor;
 event::ConnectionPtr connection;
 std::unique_ptr<ros::NodeHandle> node;
 ros::Publisher publisher;
 std::mt19937 random{42};
 std::normal_distribution<double> noise{0.0,0.01};
 public:
 void Load(sensors::SensorPtr p,sdf::ElementPtr) override {
  sensor=std::dynamic_pointer_cast<sensors::RaySensor>(p);
  if(!sensor||!ros::isInitialized()){gzerr<<"LIO lidar needs ray sensor + ROS\n";return;}
  node.reset(new ros::NodeHandle("/mineguard/lidar"));
  publisher=node->advertise<sensor_msgs::PointCloud2>("points",2);
  connection=sensor->ConnectUpdated(std::bind(&MineGuardLioLidar::Update,this));sensor->SetActive(true);
 }
 void Update(){
  if(!publisher.getNumSubscribers())return;
  int nx=sensor->RangeCount(),ny=sensor->VerticalRangeCount();
  if(nx!=sensor->RayCount()||ny!=sensor->VerticalRayCount())return;
  sensor_msgs::PointCloud2 m;auto now=sensor->LastUpdateTime();
  m.header.stamp=ros::Time(now.sec,now.nsec)-ros::Duration(0.000001);m.header.frame_id="lio_lidar";
  sensor_msgs::PointCloud2Modifier mod(m);
  mod.setPointCloud2Fields(6,"x",1,sensor_msgs::PointField::FLOAT32,"y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32,"intensity",1,sensor_msgs::PointField::FLOAT32,"ring",1,sensor_msgs::PointField::UINT16,"time",1,sensor_msgs::PointField::FLOAT32);
  mod.resize(nx*ny);
  sensor_msgs::PointCloud2Iterator<float>x(m,"x"),y(m,"y"),z(m,"z"),intensity(m,"intensity"),t(m,"time");
  sensor_msgs::PointCloud2Iterator<uint16_t>ring(m,"ring");int count=0;
  // Acquisition order is immaterial for a simultaneous snapshot; retain rings.
  for(int j=0;j<ny;++j)for(int i=0;i<nx;++i){
   double r=sensor->Range(j*nx+i);
   if(!std::isfinite(r)||r<0.3||r>=sensor->RangeMax()-1e-4)continue;
   r+=noise(random);if(r<.3)continue;
   double az=sensor->AngleMin().Radian()+i*(sensor->AngleMax().Radian()-sensor->AngleMin().Radian())/(nx-1);
   double el=sensor->VerticalAngleMin().Radian()+j*(sensor->VerticalAngleMax().Radian()-sensor->VerticalAngleMin().Radian())/(ny-1);
   *x=r*cos(el)*cos(az);*y=r*cos(el)*sin(az);*z=r*sin(el);*intensity=100;*ring=j;*t=0.000001f;
   ++x;++y;++z;++intensity;++ring;++t;++count;
  }
  mod.resize(count);m.height=1;m.width=count;m.is_dense=true;publisher.publish(m);
 }
};
GZ_REGISTER_SENSOR_PLUGIN(MineGuardLioLidar)
}

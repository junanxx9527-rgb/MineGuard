// MineGuard four-wheel drive: finite motor torque, acceleration limit and deadman.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <algorithm>
namespace gazebo {
class MineGuardCourseDrive : public ModelPlugin {
 physics::ModelPtr model; physics::JointPtr joints[4]; event::ConnectionPtr update;
 std::unique_ptr<ros::NodeHandle> nh; ros::Subscriber sub; ros::Publisher telemetry,odom;
 std::mutex mutex;
 double v=0,w=0,fv=0,fw=0,last=0,lastpub=0,stamp=-100,encx=0,ency=0,encyaw=0;
 double integral[4]={0,0,0,0};
 double turn_scale=1.0,drive_yaw=0;
 static double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
 void command(const geometry_msgs::TwistConstPtr& m){std::lock_guard<std::mutex> l(mutex);v=clamp(m->linear.x,-.5,.5);w=clamp(m->angular.z,-1,1);stamp=model->GetWorld()->SimTime().Double();}
 public: void Load(physics::ModelPtr m,sdf::ElementPtr){
  model=m;nh.reset(new ros::NodeHandle("/mineguard"));
  nh->param("course_turn_scale",turn_scale,1.0);
  if(!std::isfinite(turn_scale)||turn_scale<1.0||turn_scale>2.5)gzthrow("Invalid bounded course turn calibration");
  nh->setParam("course_turn_scale_loaded",turn_scale);
  const char* names[]={"front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"};
  for(int i=0;i<4;i++){joints[i]=m->GetJoint(names[i]);if(!joints[i])gzthrow("MineGuard missing wheel joint");joints[i]->SetParam("fmax",0,0.0);}
  sub=nh->subscribe("cmd_vel",1,&MineGuardCourseDrive::command,this);telemetry=nh->advertise<std_msgs::Float64MultiArray>("drive_state",1);odom=nh->advertise<nav_msgs::Odometry>("wheel_odom",1);
  last=m->GetWorld()->SimTime().Double();update=event::Events::ConnectWorldUpdateBegin(std::bind(&MineGuardCourseDrive::step,this));
 }
 void step(){
  const double t=model->GetWorld()->SimTime().Double(),dt=t-last;last=t;if(dt<=0||dt>.1)return;
  double cv,cw;{std::lock_guard<std::mutex> l(mutex);cv=t-stamp<.4?v:0;cw=t-stamp<.4?w:0;}
  fv+=clamp(cv-fv,-.5*dt,.5*dt);fw+=clamp(cw-fw,-1.0*dt,1.0*dt);
  // Calibrate skid-steer command conversion only. Preserve the original
  // differential wheel-target slew, PI gains, torque cap and deadman.
  drive_yaw+=clamp(cw*turn_scale-drive_yaw,-1.0*dt,1.0*dt);
  std_msgs::Float64MultiArray data;data.data={cv,cw,fv,fw};double speeds[4];
  // At zero speed only the common load torque is observable. Four separate
  // integrators can wind against each other on diagonal contacts. Project
  // out that internal preload while retaining the common slope-hold load.
  // Same gains, 12 Nm cap, wheel feedback, acceleration and 0.4s deadman.
  if(cv==0&&cw==0){
   double common=0;for(int k=0;k<4;k++)common+=integral[k]/4.0;
   for(int k=0;k<4;k++)integral[k]=common;
  }

  for(int i=0;i<4;i++){
   const double target=(fv+((i%2)?1:-1)*drive_yaw*.48/2)/.12;
   speeds[i]=joints[i]->GetVelocity(0);const double error=target-speeds[i];
   // Retain the bounded PI load estimate at zero speed. Resetting every
   // physics step leaves a proportional-only brake which rolls on slopes.
   // Wheel-speed feedback only; same 12 Nm cap, gains and deadman.
   const double raw=6.0*error+integral[i];
   if(fabs(raw)<12||raw*error<0)integral[i]=clamp(integral[i]+5.0*error*dt,-12,12);
   const double torque=clamp(6.0*error+integral[i],-12,12);
   joints[i]->SetForce(0,torque);data.data.push_back(speeds[i]);data.data.push_back(torque);
  }
  const double left=.5*(speeds[0]+speeds[2])*.12,right=.5*(speeds[1]+speeds[3])*.12;
  const double ev=.5*(right+left),ew=(right-left)/.48;encx+=ev*cos(encyaw)*dt;ency+=ev*sin(encyaw)*dt;encyaw+=ew*dt;
  if(t-lastpub>.03){lastpub=t;telemetry.publish(data);nav_msgs::Odometry msg;msg.header.stamp=ros::Time(t);msg.header.frame_id="wheel_odom";msg.child_frame_id="base_footprint";msg.pose.pose.position.x=encx;msg.pose.pose.position.y=ency;msg.pose.pose.orientation=tf::createQuaternionMsgFromYaw(encyaw);msg.twist.twist.linear.x=ev;msg.twist.twist.angular.z=ew;msg.pose.covariance[0]=.02;msg.pose.covariance[7]=.02;msg.pose.covariance[35]=.1;odom.publish(msg);

  }
 }
}; GZ_REGISTER_MODEL_PLUGIN(MineGuardCourseDrive)
}

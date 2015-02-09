#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sar_localization/Imu.h"
#include "sar_localization/Csi.h"

#include "/opt/eigen/Eigen/Eigen"
#include <vector>
#include <complex>
//Global variables for data processing
#define PI 3.1415926

using namespace std;

vector<complex> CSI1;		//CSI from antenna 1
vector<complex> CSI2;		//CSI from antenna 2
vector<pair<double, double> > orientation;	//pitch, yaw
double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
int size = 100;					//data processing size


// %Tag(CALLBACK)%
void imuCallback(const sar_localization::Imu::ConstPtr& msg)
{
 	ROS_INFO("[Seq:%d,time:%.3f,frame_id:%s]", msg->header.seq, msg->header.stamp.toNSec()*1e-6, msg->header.frame_id.c_str() );
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
	ROS_INFO("[Seq:%d,time:%.3f,csi1:%.3f %.3fi,csi2:%.3f %.3fi]", msg->header.seq, msg->header.stamp.toNSec()*1e-6, msg->csi1_real, msg->csi1_image, msg->csi2_real, msg->csi2_image);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
  ros::Subscriber sub1 = n.subscribe("imu_csi", 1000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("imu_csi", 1000, csiCallback);
// %EndTag(SUBSCRIBER)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}

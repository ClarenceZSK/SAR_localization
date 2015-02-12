#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sar_localization/Imu.h"
#include "sar_localization/Csi.h"

#include "/opt/eigen/Eigen/Dense"
#include <vector>
#include <complex>
#include <math.h>
#include <assert.h>
//#include <tuple>
//Global variables for data processing
#define PI 3.1415926
#define sizeLimit 50
#define profileLimit 20

using namespace std;
using namespace Eigen;

//vector<complex<double> > CSI1;		//CSI from antenna 1
//vector<complex<double> > CSI2;		//CSI from antenna 2
complex<double>  CSI1[sizeLimit];
complex<double>  CSI2[sizeLimit];
//vector<pair<double, double> > orientation;	//pitch, yaw
pair<double, double> orientation[sizeLimit];
double t_stamp_csi;			//time stamp of csi
double t_stamp_imu;			//time stamp of imu
bool csi_ready = false;
bool imu_ready = false;
double pitch;
double yaw;
complex<double> csi1;
complex<double> csi2;

Eigen::MatrixXd multipathProfile(360,360);

double landa = 0.06;					//The aperture size is 6cm
double r = 0.079;							//The antenna interval is 6cm

int dataIndex = 0;
int count_d = 0;
bool start = false;

//for debug
int preRow = 0, preCol = 0;

double PowerCalculation(double alpha, double beta)
{
	//int n = orientation.size();
	//printf("Size is %d\n",n);
	complex<double> avgCsiHat (0, 0);

	for(int i = 0; i < sizeLimit; ++i)
	{
		double theta = 2*PI/landa*r*cos(alpha-orientation[i].first)*sin(beta-orientation[i].second);
		double real_tmp = cos(theta);
		double image_tmp = sin(theta);
		complex<double> tmp (real_tmp, image_tmp);
		avgCsiHat += CSI1[i]*conj(CSI2[i])*tmp;
	}
	avgCsiHat /= sizeLimit;
	double ret = avgCsiHat.real()*avgCsiHat.real() + avgCsiHat.imag()*avgCsiHat.imag();
	//printf("Power calculation: %lf\n", ret);
	return ret;
}

pair<int, int> findDirectPath()
{
	Eigen::MatrixXd avgProfile = multipathProfile/(double)count_d;
	int row, col;
	printf("Count:%d,preMax:%lf,",count_d,avgProfile(preRow,preCol) );
	double maxV = avgProfile.maxCoeff(&row, &col);
	//printf("Mul-pro max:%lf, avg-pro max:%lf\n",multipathProfile(row,col), avgProfile(row,col) );
	preRow = row;
	preCol = col;
	printf("max pow:%lf\n", maxV);

	return make_pair(row,col);
}

pair<int, int> SAR_Profile_3D()
{
	//Align csi and imu data
	if(csi_ready && imu_ready)
	{
		csi_ready = false;
		imu_ready = false;
		double timeDifference = fabs(t_stamp_csi-t_stamp_imu);
		//printf("T_D:%lf, ", timeDifference);
		orientation[dataIndex % sizeLimit] = make_pair(yaw, pitch) ;
		CSI1[dataIndex % sizeLimit] = csi1;
		CSI2[dataIndex % sizeLimit] = csi2;
		if(dataIndex > 0 && dataIndex % sizeLimit == 0 && !start)
		{
			printf("Start!\n");
			start = true;
		}
		if(start)
		{
			printf("T_D:%lf, ", timeDifference);
			start = false;
			++count_d;

			//When csi and imu data vectors reach size limit, start angle generati    on
			double resolution = 1;      //search resolution
      double power = 0;
      pair<double, double> result;
			
      for(double alpha = 0; alpha < 360; alpha += resolution)
      {
        for(double beta = 0; beta < 360; beta += resolution)
	      {
					double alpha_r = alpha*2*PI/360;
					double beta_r = beta*2*PI/360;
        	double powtmp = PowerCalculation(alpha_r, beta_r);
					//printf("pow:%lf\n", powtmp);
					//multipathProfile(alpha, beta) *= 0.5;
					multipathProfile(alpha, beta) += powtmp;
      	}
			}
			pair<int, int> directPath = findDirectPath();
			++dataIndex;
      return directPath;
		}
		
		//else if(orientation.size() < sizeLimit)
		//{
		//	return make_pair(-1,-1);
		//}
		++dataIndex;
	}

	return make_pair(-1,-1);
}

// %Tag(CALLBACK)%
void imuCallback(const sar_localization::Imu::ConstPtr& msg)
{ 
  t_stamp_imu = msg->header.stamp.toNSec()*1e-6;
  pitch = msg->pitch;
  yaw = msg->yaw;
	pitch = pitch*2*PI/360;
	yaw = yaw*2*PI/360;
  imu_ready = true;
}

void csiCallback(const sar_localization::Csi::ConstPtr& msg)
{
  t_stamp_csi = msg->header.stamp.toNSec()*1e-6;
  complex<double> csi1tmp (msg->csi1_real, msg->csi1_image);
  csi1 = csi1tmp; 
  complex<double> csi2tmp (msg->csi2_real, msg->csi2_image);
  csi2 = csi2tmp; 
  csi_ready = true;
}   
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
	multipathProfile.setZero();
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("imu", 1000, imuCallback);
	ros::Subscriber sub2 = n.subscribe("csi", 1000, csiCallback);
	// %Tag(SPIN)%
	while (n.ok())
  {
	  ros::spinOnce();
		//do something
		pair<int, int> angle = SAR_Profile_3D();
		if(angle.first > 0)
		{
			printf("Alpha:%d, Beta:%d\n", angle.first, angle.second);
		}

	}
	// %EndTag(SPIN)%
  return 0;
}

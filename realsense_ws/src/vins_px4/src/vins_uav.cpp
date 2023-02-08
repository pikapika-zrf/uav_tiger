#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" //??
#include "std_msgs/Int32.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>



using namespace std;
//using namespace Eigen;

float pose_uav[3]={0,0,0};

ros::Publisher vision_pub;

void uwb_to_fcu();

void SubCameraPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry cmaera_pose_data;
	cmaera_pose_data = *msg;
		
		pose_uav[0] =  cmaera_pose_data.pose.pose.position.y;
		pose_uav[1] =  cmaera_pose_data.pose.pose.position.x;
		pose_uav[2] =  cmaera_pose_data.pose.pose.position.z;
	
}


void uwb_to_fcu()
{
	geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pose_uav[0];
        vision.pose.position.y = pose_uav[1];
        vision.pose.position.z = pose_uav[2];

        //vision.pose.orientation.x = q_uwb.x();
        //vision.pose.orientation.y = q_uwb.y();
        //vision.pose.orientation.z = q_uwb.z();
        //vision.pose.orientation.w = q_uwb.w();

	vision.header.stamp = ros::Time::now();
	vision_pub.publish(vision);
	
}



int main(int argc,char **argv)
{
	ros::init(argc,argv,"uwb0");
	ros::NodeHandle n("");
	ros::NodeHandle nh("~");

	ros::Subscriber sub_camera_pose = n.subscribe("/vins_estimator/imu_propagate",100,SubCameraPose);
	vision_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		ros::spinOnce();
		uwb_to_fcu();
		loop_rate.sleep();
	}  //无法在回调函数里发布话题，报错函数里没有定义vel_pub!只能在main里面发布了

	return 0;
}

//
// Created by bpeer on 17-10-19
//
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "bprobot/msg_Q_pose.h"
#include "bprobot/msg_F_ARM_STATUS.h"

using namespace std;

class PoseListener
{
public:
	tf::TransformListener tf_pose;
	PoseListener();
	void pubMsgData( const tf::StampedTransform& source );

	~PoseListener(){};

private:
	ros::NodeHandle nh_;
	ros::Publisher pose_pub_, pose_pub_json_;  // Q_pose
	ros::Publisher pose_pub_msg_; // msg_Q_pose
	ros::Subscriber arm_status_sub_; // msg_F_ARM_STATUS

	tf::tfMessage base2Map;
	geometry_msgs::TransformStamped pose;
	std_msgs::String base2Map_json;
	bprobot::msg_Q_pose msg_q_pose;
	int arm_status;
};

PoseListener::PoseListener()
{
	pose_pub_ = nh_.advertise< tf::tfMessage >("Q_pose", 10, true);
	pose_pub_json_ = nh_.advertise< std_msgs::String >("Q_pose_json", 10, true);
	pose_pub_msg_ = nh_.advertise< bprobot::msg_Q_pose >("msg_Q_pose", 10, true);
	/**
	 * @param: stat 1:站立状态; 2:趴下状态;
	 */
	arm_status_sub_ = nh_.subscribe<bprobot::msg_F_ARM_STATUS>("/msg_F_ARM_STATUS", 1,
	                                                           [this](const bprobot::msg_F_ARM_STATUSConstPtr &armStat)
	                                                           {
		                                                           arm_status = armStat->stat;
		                                                           cout << "arm stat: " << arm_status << endl;
	                                                           });
}

void PoseListener::pubMsgData( const tf::StampedTransform& source )
{
	pose.header.frame_id = "map";
	pose.header.stamp = ros::Time::now();
	pose.child_frame_id = "base_link";

	pose.transform.translation.x = source.getOrigin().getX();
	pose.transform.translation.y = source.getOrigin().getY();
	pose.transform.translation.z = source.getOrigin().getZ();

	pose.transform.rotation.x = source.getRotation().getX();
	pose.transform.rotation.y = source.getRotation().getY();
	pose.transform.rotation.z = source.getRotation().getZ();
	pose.transform.rotation.w = source.getRotation().getW();

	double pose_th = tf::getYaw( pose.transform.rotation );
	char str[100];
	// x, y, th
	sprintf( str,"{x:%.3f,y:%.3f,th:%.3f}", pose.transform.translation.x,
	         pose.transform.translation.y, pose_th );

	// x, y, th, angle
//	sprintf( str,"{x:%.3f,y:%.3f,th:%.3f,angle:%.2f}", base2Map.transform.translation.x,
//	         base2Map.transform.translation.y, pose_th, pose_th * 180 / M_PI );
	base2Map_json.data = str;

	msg_q_pose.time = pose.header.stamp;
	msg_q_pose.x = pose.transform.translation.x;
	msg_q_pose.y = pose.transform.translation.y;
	msg_q_pose.Deg = pose_th * 180 / M_PI;
	msg_q_pose.Rad = pose_th;
	msg_q_pose.ArmS = arm_status;

	cout << "done ." << endl;

	pose_pub_msg_.publish( msg_q_pose );  // msg_Q_pose
	pose_pub_json_.publish( base2Map_json ); // Q_pose_json

	base2Map.transforms.push_back( pose );
	pose_pub_.publish( base2Map );  // Q_pose
	base2Map.transforms.clear();  // clear data
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "pub_tf2topic_node" );

	PoseListener poseListener;  //class

	tf::StampedTransform transform;
	sleep(1);
	ros::Rate r( 10 );

	while( ros::ok() )
	{
		try
		{
			poseListener.tf_pose.lookupTransform( "/map", "/base_link", ros::Time(0), transform );

			poseListener.pubMsgData( transform );

			poseListener.tf_pose.clear();  //@todo 可以用超时判断....
			cout << "base2map tf ok... " << endl;
		}
		catch ( tf::TransformException &ex )
		{
			cout << "no tf base2map...  " << endl;
		}

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

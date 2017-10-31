//
// Created by bpeer on 17-10-19
//
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"

using namespace std;

class PoseListener
{
public:
	tf::TransformListener tf_pose;
	PoseListener();
	void copyMsgData( const tf::StampedTransform& source );

	~PoseListener(){};

private:
	ros::NodeHandle nh_;
	ros::Publisher pose_pub_, pose_pub_json_;

	geometry_msgs::TransformStamped base2Map;

	std_msgs::String base2Map_json;
};

PoseListener::PoseListener()
{
	pose_pub_ = nh_.advertise< geometry_msgs::TransformStamped >("Q_pose", 10);
	pose_pub_json_ = nh_.advertise< std_msgs::String >("Q_pose_json", 10);
}

void PoseListener::copyMsgData( const tf::StampedTransform& source )
{
	base2Map.header.frame_id = "map";
	base2Map.header.stamp = ros::Time::now();
	base2Map.child_frame_id = "base_link";

	base2Map.transform.translation.x = source.getOrigin().getX();
	base2Map.transform.translation.y = source.getOrigin().getY();
	base2Map.transform.translation.z = source.getOrigin().getZ();

	base2Map.transform.rotation.x = source.getRotation().getX();
	base2Map.transform.rotation.y = source.getRotation().getY();
	base2Map.transform.rotation.z = source.getRotation().getZ();
	base2Map.transform.rotation.w = source.getRotation().getW();

	double pose_th = tf::getYaw( source.getRotation() );
	char str[100];
	sprintf( str,"{x:%.3f,y:%.3f,th:%.3f,angle:%.2f}", base2Map.transform.translation.x,
	         base2Map.transform.translation.y, pose_th, pose_th * 180 / M_PI );
	base2Map_json.data = str;
	cout << "done ." << endl;

	pose_pub_.publish( base2Map );
	pose_pub_json_.publish( base2Map_json );
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "pub_tf2topic_node" );

	PoseListener poseListener;  //class

	ros::Rate r( 20 );

	while( ros::ok() )
	{
		tf::StampedTransform transform;
		try
		{
			poseListener.tf_pose.lookupTransform( "/map", "/base_link", ros::Time(0), transform );

			poseListener.copyMsgData( transform );

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

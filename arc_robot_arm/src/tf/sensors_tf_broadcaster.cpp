#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <math.h>

using namespace std;

void sendStaticTransform(double translation[] ,double rotation[], string child, string parent) {
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = parent;
  static_transformStamped.child_frame_id = child;
  static_transformStamped.transform.translation.x = translation[0];
  static_transformStamped.transform.translation.y = translation[1];
  static_transformStamped.transform.translation.z = translation[2];
  
	tf2::Quaternion quat;
  quat.setRPY(rotation[0], rotation[1], rotation[2]);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
	static_broadcaster.sendTransform(static_transformStamped);
}

void sendDynamicTransform(double translation[], double rotation[], string child, string parent)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent;
  transformStamped.child_frame_id = child;
  transformStamped.transform.translation.x = translation[0];
  transformStamped.transform.translation.y = translation[1];
  transformStamped.transform.translation.z = translation[2];
  
	tf2::Quaternion quat;
  quat.setRPY(rotation[0], rotation[1], rotation[2]);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  
	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensors_tf_broadcaster");
  ros::NodeHandle n;
	double camera_left_trans[3] = {-0.087, 0.02, 0.06};
	double camera_rot[3] = {0,0,0};
	
	sendStaticTransform(camera_left_trans, camera_rot, "camera", "link_3_v1_1");
	
	ros::spin();

	return 0;
}

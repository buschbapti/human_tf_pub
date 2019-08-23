#ifndef __HUMAN_TF_PUBLISHER_H__
#define __HUMAN_TF_PUBLISHER_H__

#include "ros/ros.h"
#include <vector>
#include <unordered_map>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <body_tracker_msgs/Skeleton_v2.h>

#include <tf/transform_broadcaster.h>


#include <signal.h>

class Human_tf_publisher {


	// ROS variables
protected:
	ros::NodeHandle nh_;
private:
	ros::Rate loop_rate_;

	// subscribers and publishers
	ros::Subscriber body_tracking_skeleton_sub_;

	// Class variables
protected:
	// time step of node
	double dt_;


private:

	static Human_tf_publisher* me_;
	bool stop_ = false;

    tf::TransformBroadcaster joint_tf_broadcaster_;

    std::vector<tf::Transform> joints_transform_;
    std::vector<std::string> joints_tf_names_;

    std::string tf_origin_frame_name_ = "kinect_frame";
    std::string tf_prefix = "kinect/human";

public:
	Human_tf_publisher(ros::NodeHandle &n,
		double frequency,
		std::string input_joints_topic_name,
    	std::vector<std::string> output_joints_tf_names);

	bool Init();

	void Run();

private:

	void process_skeleton(body_tracker_msgs::Skeleton_v2::ConstPtr msg);

	void update_joint_tf(tf::Transform& joint_tf,const geometry_msgs::Pose msg,std::string tf_name);

    geometry_msgs::Pose get_pose_of_joint(body_tracker_msgs::Skeleton_v2::ConstPtr msg,int joint_num);

	// Function called when the node is killed through CTRL + C
	static void stopNode(int sig);


protected:

};


#endif //__HUMAN_TF_PUBLISHER_H__
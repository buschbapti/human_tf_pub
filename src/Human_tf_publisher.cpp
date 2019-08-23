#include "Human_tf_publisher.h"

#define SUB_BUFFER_SIZE 1000

//create pointer to self for CTRL + C catching
Human_tf_publisher* Human_tf_publisher::me_ = NULL;




//====================Initialisation and Run functions==========================

Human_tf_publisher::Human_tf_publisher(ros::NodeHandle &n,
		double frequency,
		std::string input_joints_topic_name,
    	std::vector<std::string> output_joints_tf_names
	)
	:nh_(n), loop_rate_(frequency), joints_tf_names_(output_joints_tf_names)
{

	me_ = this;

	// interruption set up to catch CTRL + C
	signal(SIGINT,Human_tf_publisher::stopNode);

	//ROS Topic initialization
	//sub_real_pose_ = nh_.subscribe($topic_name, SUB_BUFFER_SIZE,
	//		&Human_tf_publisher::callback, this,ros::TransportHints().reliable().tcpNoDelay());

	// Publish tracked person upper body skeleton for advanced uses
    body_tracking_skeleton_sub_ = nh_.subscribe(input_joints_topic_name,1,
      											&Human_tf_publisher::process_skeleton,this,
      											ros::TransportHints().reliable().tcpNoDelay());

    joints_transform_.resize(output_joints_tf_names.size());

	//pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(
	//		output_vel_topic_name, 1000, 1);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The node is ready.");
	}


	Init();
	ROS_INFO_STREAM("Human_tf_publisher.CPP: Human_tf_publisher node is created at: "
			 << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}// end constructor


bool Human_tf_publisher::Init() {

	return true;
}//end Init

void Human_tf_publisher::Run() {

	while (!stop_ && ros::ok()) {

		ros::spinOnce();

		loop_rate_.sleep();
	}

	ros::shutdown();

}

void Human_tf_publisher::process_skeleton(body_tracker_msgs::Skeleton_v2::ConstPtr msg){

	
	for(int i = 0; i< joints_tf_names_.size();i++){

		std::string joint = joints_tf_names_[i];
		auto joint_msg = get_pose_of_joint(msg,i);
		std::string tf_full_name = tf_prefix + "/" + joint;

		update_joint_tf(joints_transform_[i],joint_msg,tf_full_name);
	}
}


void Human_tf_publisher::update_joint_tf(tf::Transform& joint_tf,const geometry_msgs::Pose msg,std::string tf_name){

	joint_tf.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z) );
    joint_tf.setRotation( tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    joint_tf_broadcaster_.sendTransform(tf::StampedTransform(joint_tf, ros::Time::now(), tf_origin_frame_name_, tf_name));

}

geometry_msgs::Pose Human_tf_publisher::get_pose_of_joint(body_tracker_msgs::Skeleton_v2::ConstPtr msg,int joint_num){
	switch (joint_num){
        case (0) : return msg->joint_position_head;

        case (1) : return msg->joint_position_neck;

        case (2) : return msg->joint_position_shoulder_center;

        case (3) : return msg->joint_position_spine;

        case (4) : return msg->joint_position_base;

        case (5) : return msg->joint_position_left_shoulder;

        case (6) : return msg->joint_position_left_elbow;

        case (7) : return msg->joint_position_left_wrist;

        case (8) : return msg->joint_position_left_hip;

        case (9) : return msg->joint_position_left_knee;

        case (10) : return msg->joint_position_left_ankle;

        case (11) : return msg->joint_position_right_shoulder;

        case (12) : return msg->joint_position_right_elbow;

        case (13) : return msg->joint_position_right_wrist;

        case (14) : return msg->joint_position_right_hip;

        case (15) : return msg->joint_position_right_knee;

        case (16) : return msg->joint_position_right_ankle;

		default: return msg->joint_position_base;
	}
}

//====================END: Initialisation and Run functions=====================

void Human_tf_publisher::stopNode(int sig)
{
	ROS_INFO("Catched the ctrl C");
	me_->stop_ = true;
}

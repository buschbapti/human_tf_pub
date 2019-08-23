#include "Human_tf_publisher.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_tf_publisher");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  //inputs
  std::string input_joints_topic_name = "body_tracker/skeleton_v2";

  //outputs
  std::vector<std::string> output_joints_tf_names = {"head", "neck", "shoulder_center",
        "spine", "base", "left_shoulder", "left_elbow", "left_wrist", "left_hip",
        "left_knee", "left_ankle", "right_shoulder", "right_elbow", "right_wrist",
        "right_hip", "right_knee", "right_ankle"};

  /*if (!nh.getParam("input_rob_pose_topic_name", input_rob_pose_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }*/


  Human_tf_publisher my_human_tf_publisher(
    nh,
    frequency,
    input_joints_topic_name,
    output_joints_tf_names
    );

  my_human_tf_publisher.Run();

  return 0;
}

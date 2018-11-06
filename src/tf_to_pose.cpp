#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher rosbot_map_pub;
geometry_msgs::PoseStamped rosbot_pose;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_to_pose");
  tf::TransformListener listener;

  ros::NodeHandle node("~");
  std::string parent_frame;
  std::string child_frame;
  std::string out_topic;
  node.param<std::string>("parent_frame", parent_frame, "/map");
  node.param<std::string>("child_frame", child_frame, "/base_link");
  node.param<std::string>("out_topic", out_topic, "/rosbot_on_map_pose");

  rosbot_map_pub = node.advertise<geometry_msgs::PoseStamped>(out_topic, 1);

  rosbot_pose.header.frame_id = parent_frame;
  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get transform between '%s' and '%s', will retry every second", parent_frame.c_str(), child_frame.c_str());
      ros::Duration(1.0).sleep();
    }

    rosbot_pose.pose.position.x = transform.getOrigin().x();
    rosbot_pose.pose.position.y = transform.getOrigin().y();
    rosbot_pose.pose.position.z = transform.getOrigin().z();
    rosbot_pose.pose.orientation.x = transform.getRotation().x();
    rosbot_pose.pose.orientation.y = transform.getRotation().y();
    rosbot_pose.pose.orientation.z = transform.getRotation().z();
    rosbot_pose.pose.orientation.w = transform.getRotation().w();

    rosbot_map_pub.publish(rosbot_pose);

    rate.sleep();
  }
  return 0;
};

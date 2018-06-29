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

  ros::NodeHandle node;
  rosbot_map_pub = node.advertise<geometry_msgs::PoseStamped>("/rosbot_on_map_pose", 1);

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("Proceed with tf");
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

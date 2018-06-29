#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
nav_msgs::Odometry odom;

void poseCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0));
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = msg->pose.position.z;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_to_tf_transform");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/pose", 10, &poseCallback);
    odom_pub = node.advertise<nav_msgs::Odometry>("/odom", 1);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    //odom.pose.covariance

    ros::spin();
    return 0;
};

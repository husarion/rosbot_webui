#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <MapAsImageProvider.h>
#include <tf/transform_listener.h>

using namespace std;

MapAsImageProvider *map_image_provider;
int32_t tile_width;
int32_t tile_height;
std::string parent_frame;
std::string child_frame;

void map_zoom_callback(const std_msgs::Int16 &scale)
{
  map_image_provider->setScale((float)scale.data / 2500);
}

int main(int argc, char **argv)
{
  tf::StampedTransform transform;

  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle n("~");
  n.param<int32_t>("tile_width", tile_width, INITIAL_TILE_SIZE_X);
  n.param<int32_t>("tile_height", tile_height, INITIAL_TILE_SIZE_Y);
  n.param<std::string>("parent_frame", parent_frame, "/map");
  n.param<std::string>("child_frame", child_frame, "/base_link");

  ros::Rate loop_rate(50);
  ROS_INFO("Init MapAsImageProvider object");
  map_image_provider = new MapAsImageProvider(n, tile_width, tile_height);
  ros::Subscriber map_zoom_sub = n.subscribe("/map_zoom", 1, map_zoom_callback);
  tf::TransformListener listener;

  while (ros::ok())
  {
    ros::spinOnce();
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get transform between '%s' and '%s', will retry every second", parent_frame.c_str(), child_frame.c_str());
      ros::Duration(1.0).sleep();
    }

    loop_rate.sleep();
    map_image_provider->publishFullMap();
    map_image_provider->publishMapTile();
  }
  return 0;
}

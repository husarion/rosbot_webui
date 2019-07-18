#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#ifndef MAP_AS_IMAGE_PROVIDER_H
#define MAP_AS_IMAGE_PROVIDER_H

#define INITIAL_TILE_SIZE_X 512
#define INITIAL_TILE_SIZE_Y 512
#define INITIAL_MAP_SIZE_X 768
#define INITIAL_MAP_SIZE_Y 768
#define INITIAL_FULL_MAP_DELAY 1
#define INITIAL_TILE_DELAY 0.1
#define DEFAULT_MAP_SCALE 0.01

class MapAsImageProvider
{
private:
    ros::NodeHandle node_handle;
    image_transport::ImageTransport *image_transport_;
    image_transport::Publisher image_transport_publisher_full_;
    image_transport::Publisher image_transport_publisher_tile_;

    nav_msgs::OccupancyGrid currentMap;

    ros::Subscriber map_sub_;
    cv_bridge::CvImage cv_img_full_;
    cv_bridge::CvImage cv_img_tile_;

    ros::Time lastMapUpdate;
    ros::Duration fullMapDelay;
    ros::Time lastTileUpdate;
    ros::Duration mapTileDelay;

    int spacing;
    float map_scale;

    float robot_position_x;
    float robot_position_y;

    void poseUpdate(const geometry_msgs::PoseStampedConstPtr &pose);
    void mapUpdate(const nav_msgs::OccupancyGridConstPtr &map);
    void tileUpdate();
    int8_t getCellOccupancy(float x, float y);

public:
    MapAsImageProvider(ros::NodeHandle nh, uint16_t tile_width = INITIAL_TILE_SIZE_X, uint16_t tile_height = INITIAL_TILE_SIZE_Y);
    ~MapAsImageProvider();
    void publishFullMap(bool force = false);
    void publishMapTile(bool force = false);
    void setScale(float scale = DEFAULT_MAP_SCALE);
    void updateRobotPosition(float x, float y);
};

#endif
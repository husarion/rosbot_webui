#include "MapAsImageProvider.h"

MapAsImageProvider::MapAsImageProvider(ros::NodeHandle nh, uint16_t tile_width, uint16_t tile_height)
{
    node_handle = nh;

    image_transport_ = new image_transport::ImageTransport(node_handle);
    image_transport_publisher_full_ = image_transport_->advertise("/map_image/full", 1);
    image_transport_publisher_tile_ = image_transport_->advertise("/map_image/tile", 1);

    map_sub_ = node_handle.subscribe("/map", 1, &MapAsImageProvider::mapUpdate, this);

    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img_full_.image = cv::Mat(INITIAL_MAP_SIZE_Y, INITIAL_MAP_SIZE_X, CV_8U, 127);
    lastMapUpdate = ros::Time::now();
    fullMapDelay = ros::Duration(INITIAL_FULL_MAP_DELAY);
    lastTileUpdate = ros::Time::now();
    mapTileDelay = ros::Duration(INITIAL_TILE_DELAY);

    cv_img_tile_.header.frame_id = "map_image";
    cv_img_tile_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img_tile_.image = cv::Mat(tile_height, tile_width, CV_8U, 127);

    robot_position_x = 0;
    robot_position_y = 0;

    setScale(DEFAULT_MAP_SCALE);

    ROS_INFO("Map to Image node started.");
}

MapAsImageProvider::~MapAsImageProvider()
{
    delete image_transport_;
}

void MapAsImageProvider::setScale(float scale)
{
    ROS_INFO("New scale is %f", scale);
    map_scale = scale;
    spacing = (int)round(map_scale / currentMap.info.resolution);
}

void MapAsImageProvider::publishFullMap(bool force)
{
    if (force)
    {
        image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
        lastMapUpdate = ros::Time::now();
    }
    else if (lastMapUpdate + fullMapDelay < ros::Time::now())
    {
        image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
        lastMapUpdate = ros::Time::now();
    }
}

void MapAsImageProvider::publishMapTile(bool force)
{
    if (force)
    {
        tileUpdate();
        image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
        lastTileUpdate = ros::Time::now();
    }
    else if (lastTileUpdate + mapTileDelay < ros::Time::now())
    {
        tileUpdate();
        image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
        lastTileUpdate = ros::Time::now();
    }
}

void MapAsImageProvider::updateRobotPosition(float x, float y)
{
    robot_position_x = x;
    robot_position_y = y;
}

void MapAsImageProvider::mapUpdate(const nav_msgs::OccupancyGridConstPtr &map)
{
    if ((map->info.width < 3) || (map->info.height < 3))
    {
        ROS_WARN("Map size is only x: %d,  y: %d . Not running map to image conversion", map->info.width, map->info.height);
        return;
    }
    currentMap.header = map->header;
    currentMap.info = map->info;
    currentMap.data = map->data;

    // resize cv image if it doesn't have the same dimensions as the map
    if ((cv_img_full_.image.rows != map->info.height) && (cv_img_full_.image.cols != map->info.width))
    {
        cv_img_full_.image = cv::Mat(map->info.height, map->info.width, CV_8U);
    }

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = cv_img_full_.image.rows - 1;
    for (int y = size_y_rev; y >= 0; --y)
    {
        int idx_map_y = cv_img_full_.image.cols * y;
        int idx_img_y = cv_img_full_.image.cols * (cv_img_full_.image.rows - y);

        for (int x = 0; x < cv_img_full_.image.cols; ++x)
        {
            int idx = idx_img_y + x;
            switch (currentMap.data[idx_map_y + x])
            {
            case -1:
                cv_img_full_.image.data[idx] = 127;
                break;
            case 0:
                cv_img_full_.image.data[idx] = 255;
                break;
            case 100:
                cv_img_full_.image.data[idx] = 0;
                break;
            }
        }
    }
    publishFullMap(true);
}

int8_t MapAsImageProvider::getCellOccupancy(float x, float y)
{
    int32_t grid_x = (x - (int)currentMap.info.origin.position.x) / currentMap.info.resolution;
    int32_t grid_y = (y - (int)currentMap.info.origin.position.y) / currentMap.info.resolution;
    if (grid_x >= 0 && grid_x < currentMap.info.width && grid_y >= 0 && grid_y < currentMap.info.height)
    {
        return currentMap.data[grid_y * currentMap.info.width + grid_x];
    }
    else
    {
        return -1;
    }
}

void MapAsImageProvider::tileUpdate()
{

    int8_t currentPoint;
    for (int x = 0; x < cv_img_tile_.image.cols; x++)
    {
        for (int y = 0; y < cv_img_tile_.image.rows; y++)
        {
            int idx = (cv_img_tile_.image.rows - y) * cv_img_tile_.image.cols + x;
            if (map_scale > currentMap.info.resolution)
            {
                int tmpPoint;
                currentPoint = -1;
                for (int dx = 0; dx < spacing; dx++)
                {
                    for (int dy = 0; dy < spacing; dy++)
                    {

                        tmpPoint = getCellOccupancy(
                            robot_position_x + ((x - cv_img_tile_.image.cols / 2) * map_scale + dx * currentMap.info.resolution),
                            robot_position_y + ((y - cv_img_tile_.image.rows / 2) * map_scale + dy * currentMap.info.resolution));
                        if (tmpPoint > currentPoint)
                        {
                            currentPoint = tmpPoint;
                        }
                    }
                }
            }
            else
            {
                currentPoint = getCellOccupancy(
                    robot_position_x + ((x - cv_img_tile_.image.cols / 2) * map_scale),
                    robot_position_y + ((y - cv_img_tile_.image.rows / 2) * map_scale));
            }
            switch (currentPoint)
            {
            case -1:
                cv_img_tile_.image.data[idx] = 127;
                break;
            case 0:
                cv_img_tile_.image.data[idx] = 255;
                break;
            case 100:
                cv_img_tile_.image.data[idx] = 0;
                break;
            }
        }
    }
}

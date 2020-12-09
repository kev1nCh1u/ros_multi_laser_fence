
/**************************************************************************************************************************************
 * Fence
 * 20201208 by KevinChiu
***************************************************************************************************************************************/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vector>

class KevinFence
{
private:
  ros::NodeHandle node;

  ros::Timer timer;
  ros::Publisher vis_pub;
  ros::Subscriber scan_sub;
  ros::Subscriber merged_cloud_sub;

  ros::Time now;

  void drawSquare(int id, float x1, float y1, float x2, float y2, int color, ros::Publisher vis_pub);
  void scanSubCallback(const sensor_msgs::LaserScan &msg);
  void mergedCloudSubCallback(const sensor_msgs::PointCloud2 &msg);
  void timerCallback(const ros::TimerEvent &);

  tf::TransformListener listener1;
  tf::TransformListener listener2;
  std::vector<tf::StampedTransform> transformVec;

  float fenceRange;


  typedef struct
  {
    float x;
    float y;
  }pos_t;
  pos_t origin_pos[2];

public:
  KevinFence(/* args */);
  ~KevinFence();
};

KevinFence::KevinFence(/* args */)
{
  now = ros::Time::now();

  timer = node.createTimer(ros::Duration(1), &KevinFence::timerCallback, this);
  vis_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0, this);
  scan_sub = node.subscribe("scan", 1000, &KevinFence::scanSubCallback, this);
  merged_cloud_sub = node.subscribe("merged_cloud", 1000, &KevinFence::mergedCloudSubCallback, this);

  fenceRange = 0.5;

  transformVec.resize(2);
  try
  {
    listener1.waitForTransform("/base_link", "/laser1", now, ros::Duration(3.0));
    listener1.lookupTransform("/base_link", "/laser1", ros::Time(0), transformVec[0]);
    listener2.waitForTransform("/base_link", "/laser2", now, ros::Duration(3.0));
    listener2.lookupTransform("/base_link", "/laser2", ros::Time(0), transformVec[1]);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  origin_pos[0].x = transformVec[0].getOrigin().x();
  origin_pos[0].y = transformVec[0].getOrigin().y();
  origin_pos[1].x = transformVec[1].getOrigin().x();
  origin_pos[1].y = transformVec[1].getOrigin().y();

  // std::cout << " x1:" << origin_pos[0].x << " y1:" << origin_pos[0].y << " x2:" << origin_pos[1].x << " y2:" << origin_pos[1].y << std::endl;
}

KevinFence::~KevinFence()
{
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
void KevinFence::timerCallback(const ros::TimerEvent &)
{
  drawSquare(0, origin_pos[0].x, origin_pos[0].y, origin_pos[1].x, origin_pos[1].y, 4, vis_pub);
  // drawSquare(1, origin_pos[0].x + fenceRange, origin_pos[0].y + fenceRange, origin_pos[1].x - fenceRange, origin_pos[1].y - fenceRange, 2, vis_pub);
}

/****************************************************************************************************************************************
* subscribe scan
* ***************************************************************************************************************************************/
void KevinFence::scanSubCallback(const sensor_msgs::LaserScan &msg)
{

  // std::cout << " scanSubCallback: ";
  // for (int i = 30; i < 50; i++)
  // {
  //   std::cout << msg.ranges[i] << "\t";
  // }
  // std::cout << std::endl;
}

/****************************************************************************************************************************************
* subscribe mergedCloudSub
* ***************************************************************************************************************************************/
void KevinFence::mergedCloudSubCallback(const sensor_msgs::PointCloud2 &msg)
{

  // std::cout << " mergedCloudSubCallback: ";
  // for (int i = 30; i < 50; i++)
  // {
  //   std::cout << msg.ranges[i] << "\t";
  // }
  // std::cout << std::endl;


	sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
  int flag = 0;
	for (int i=0; i<out_pointcloud.points.size(); i++)
  {
		// std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << std::endl;

    // if(out_pointcloud.points[i].x < origin_pos[0].x + fenceRange && out_pointcloud.points[i].x > origin_pos[1].x - fenceRange)
    if(out_pointcloud.points[i].x > origin_pos[0].x + fenceRange || out_pointcloud.points[i].x < origin_pos[1].x - fenceRange)
      flag = 1;
    // if(out_pointcloud.points[i].y < origin_pos[0].y + fenceRange && out_pointcloud.points[i].y > origin_pos[1].y - fenceRange)
    if(out_pointcloud.points[i].y > origin_pos[0].y + fenceRange || out_pointcloud.points[i].y < origin_pos[1].y - fenceRange)
      flag = 1;
    std::cout << "flag: " << flag << std::endl;
    if(flag)
      drawSquare(1, origin_pos[0].x + fenceRange, origin_pos[0].y + fenceRange, origin_pos[1].x - fenceRange, origin_pos[1].y - fenceRange, 1, vis_pub);
    else
      drawSquare(1, origin_pos[0].x + fenceRange, origin_pos[0].y + fenceRange, origin_pos[1].x - fenceRange, origin_pos[1].y - fenceRange, 2, vis_pub);
    
	}
}

/****************************************************************************************************************************************
* marker draw square
* ***************************************************************************************************************************************/
void KevinFence::drawSquare(int id, float x1, float y1, float x2, float y2, int color, ros::Publisher vis_pub)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "square";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  switch (color)
  {
  case 0:
    break;
  case 1:
    marker.color.r = 1.0;
    break;
  case 2:
    marker.color.g = 1.0;
    break;
  case 3:
    marker.color.b = 1.0;
    break;
  default:
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    break;
  }

  std::vector<geometry_msgs::Point> pointMsgVec(5);
  int qqX[5] = {0, 0, 1, 1, 0};
  int qqY[5] = {0, 1, 1, 0, 0};
  float posX[2] = {x1, x2};
  float posY[2] = {y1, y2};

  for (int i = 0; i < 5; i++)
  {
    pointMsgVec[i].x = posX[qqX[i]];
    pointMsgVec[i].y = posY[qqY[i]];

    marker.points.push_back(pointMsgVec[i]);
  }

  vis_pub.publish(marker);
}

/****************************************************************************************************************************************
* main
* ***************************************************************************************************************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fence");

  KevinFence kevinFence;

  ros::spin();

  return 0;
};
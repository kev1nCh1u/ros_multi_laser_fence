
/**************************************************************************************************************************************
 * Fence
 * 20201208 by KevinChiu
***************************************************************************************************************************************/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

class KevinFence
{
private:
  ros::NodeHandle node;

  ros::Timer timer;
  ros::Publisher vis_pub;
  ros::Subscriber sub;

  ros::Time now = ros::Time::now();

  void drawSquare(int id, int x1, int y1, int x2, int y2, int color, ros::Publisher vis_pub);
  void chatterCallback(const sensor_msgs::LaserScan &msg);
  void timerCallback(const ros::TimerEvent &);

  tf::TransformListener listener1;
  tf::TransformListener listener2;
  std::vector<tf::StampedTransform> transformVec;

  int fenceRange = 2;

public:
  KevinFence(/* args */);
  ~KevinFence();
};

KevinFence::KevinFence(/* args */)
{

  timer = node.createTimer(ros::Duration(1), &KevinFence::timerCallback, this);
  vis_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0, this);
  sub = node.subscribe("scan_1", 1000, &KevinFence::chatterCallback, this);

  listener1.waitForTransform("/base_link", "/laser1", now, ros::Duration(3.0));
  listener2.waitForTransform("/base_link", "/laser2", now, ros::Duration(3.0));

  transformVec.resize(2);
  try
  {

    listener1.lookupTransform("/base_link", "/laser1", ros::Time(0), transformVec[0]);

    listener2.lookupTransform("/base_link", "/laser2", ros::Time(0), transformVec[1]);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // std::cout << " x1:" << transformVec[0].getOrigin().x() << " y1:" << transformVec[0].getOrigin().y() << " x2:" << transformVec[1].getOrigin().x() << " y2:" << transformVec[1].getOrigin().y() << std::endl;
}

KevinFence::~KevinFence()
{
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
void KevinFence::timerCallback(const ros::TimerEvent &)
{

  drawSquare(0, transformVec[0].getOrigin().x(), transformVec[0].getOrigin().y(), transformVec[1].getOrigin().x(), transformVec[1].getOrigin().y(), 4, vis_pub);
  drawSquare(1, transformVec[0].getOrigin().x() + fenceRange, transformVec[0].getOrigin().y() + fenceRange, transformVec[1].getOrigin().x() - fenceRange, transformVec[1].getOrigin().y() - fenceRange, 2, vis_pub);
}

/****************************************************************************************************************************************
* subscribe scan
* ***************************************************************************************************************************************/
void KevinFence::chatterCallback(const sensor_msgs::LaserScan &msg)
{

  std::cout << " chatterCallback: ";
  for (int i = 30; i < 50; i++)
  {
    std::cout << msg.ranges[i] << "\t";
  }
  std::cout << std::endl;
}

/****************************************************************************************************************************************
* marker draw square
* ***************************************************************************************************************************************/
void KevinFence::drawSquare(int id, int x1, int y1, int x2, int y2, int color, ros::Publisher vis_pub)
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
  int posX[2] = {x1, x2};
  int posY[2] = {y1, y2};

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
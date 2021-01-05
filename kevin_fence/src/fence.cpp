
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
#include <sound_play/SoundRequest.h>

#include <vector>

/****************************************************************************************************************************************
* KevinFence class
* ***************************************************************************************************************************************/
class KevinFence
{
private:
  // ros
  ros::NodeHandle node;
  ros::NodeHandle pnh_;

  ros::Timer timer;
  ros::Timer soundTimer;
  ros::Publisher vis_pub;
  ros::Subscriber scan_sub;
  ros::Subscriber merged_cloud_sub;
  ros::Publisher robotSound_pub;

  // fuc
  void timerCallback(const ros::TimerEvent &);
  void soundTimerCallback(const ros::TimerEvent &);
  void drawSquare(int id, double x1, double y1, double x2, double y2, int color, ros::Publisher vis_pub);
  void scanSubCallback(const sensor_msgs::LaserScan &msg);
  void mergedCloudSubCallback(const sensor_msgs::PointCloud2 &msg);
  
  // struct
  typedef struct
  {
    double x;
    double y;
  } pos_t;
  typedef struct
  {
    double height;
    double width;
  } square_t;
  typedef struct
  {
    square_t range;
    int flag;
    int lastFlag;
    int pointCount;
  } fence_t;
  
  // var
  ros::Time now;
  tf::TransformListener listener[4];
  std::vector<tf::StampedTransform> transformVec;

  int fenceLevel;
  fence_t fenceStruct[10];
  // square_t fenceRange[10];
  int fenceCount;
  // int fenceFlag;
  // int lastFenceFlag;
  sound_play::SoundRequest sound_msg;

  pos_t origin_pos[2];

public:
  KevinFence(/* args */);
  ~KevinFence();
};

/****************************************************************************************************************************************
* KevinFence KevinFence
* ***************************************************************************************************************************************/
KevinFence::KevinFence(/* args */) : pnh_("~")
{
  // ros
  now = ros::Time::now();

  timer = node.createTimer(ros::Duration(1), &KevinFence::timerCallback, this);
  soundTimer = node.createTimer(ros::Duration(3), &KevinFence::soundTimerCallback, this);
  vis_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0, this);
  scan_sub = node.subscribe("scan", 1000, &KevinFence::scanSubCallback, this);
  merged_cloud_sub = node.subscribe("merged_cloud", 1000, &KevinFence::mergedCloudSubCallback, this);
  robotSound_pub = node.advertise<sound_play::SoundRequest>("robotsound", 0, this);

  // get param
  pnh_.param<int>("fence_level", fenceLevel, 2);
  pnh_.param<double>("fence_range_height", fenceStruct[0].range.height, 1);
  pnh_.param<double>("fence_range_width", fenceStruct[0].range.width, 0.5);
  pnh_.param<int>("fence_count", fenceCount, 50);
  pnh_.param<std::string>("sound_file", sound_msg.arg, "/home/user/ros/multi_laser/src/audio_common/sound_play/sounds/ts_excuse_me_chinese.wav");

  // fence range
  for(int i = 0; i < fenceLevel; i++)
  {
    fenceStruct[i].range.height = fenceStruct[0].range.height * (i + 1);
    fenceStruct[i].range.width = fenceStruct[0].range.width * (i + 1);
  }

  // print param
  std::cout << "########## param ################" << std::endl;
  std::cout << "fence_range_height: " << fenceStruct[0].range.height << std::endl;
  std::cout << "fence_range_width: " << fenceStruct[0].range.width << std::endl;
  std::cout << "fenceCount: " << fenceCount << std::endl;
  std::cout << "sound_msg.arg: " << sound_msg.arg << std::endl;
  std::cout << "#################################" << std::endl;

  // msg set
  sound_msg.sound = -2;
  sound_msg.command = 1;
  sound_msg.volume = 1.0;
  // sound_msg.arg = "/home/user/ros/multi_laser/src/audio_common/sound_play/sounds/excuse_me_ryotsu.wav";

  // tf
  transformVec.resize(2);
  try
  {
    listener[0].waitForTransform("/base_link", "/laser1", now, ros::Duration(3.0));
    listener[0].lookupTransform("/base_link", "/laser1", ros::Time(0), transformVec[0]); // laser1 to base link
    listener[1].waitForTransform("/base_link", "/laser2", now, ros::Duration(3.0));
    listener[1].lookupTransform("/base_link", "/laser2", ros::Time(0), transformVec[1]); // laser2 to base link
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // get tf org
  origin_pos[0].x = transformVec[0].getOrigin().x();
  origin_pos[0].y = transformVec[0].getOrigin().y();
  origin_pos[1].x = transformVec[1].getOrigin().x();
  origin_pos[1].y = transformVec[1].getOrigin().y();
  // std::cout << " x1:" << origin_pos[0].x << " y1:" << origin_pos[0].y << " x2:" << origin_pos[1].x << " y2:" << origin_pos[1].y << std::endl; // debug print

  // draw
  for(int i = 0; i < fenceLevel; i++)
  {
    drawSquare(1, origin_pos[0].x + fenceStruct[i].range.height, origin_pos[0].y + fenceStruct[i].range.width, origin_pos[1].x - fenceStruct[i].range.height, origin_pos[1].y - fenceStruct[i].range.width, 2, vis_pub);
  }
}

KevinFence::~KevinFence()
{
}

/***************************************************************************************************************************************
* timer
* ***************************************************************************************************************************************/
void KevinFence::timerCallback(const ros::TimerEvent &)
{
  drawSquare(0, origin_pos[0].x, origin_pos[0].y, origin_pos[1].x, origin_pos[1].y, 4, vis_pub); // draw the car

  // draw the fence
  for(int i = 0; i < fenceLevel; i++)
  {
    if (fenceStruct[i].flag)
      drawSquare(1, origin_pos[0].x + fenceStruct[i].range.height, origin_pos[0].y + fenceStruct[i].range.width, origin_pos[1].x - fenceStruct[i].range.height, origin_pos[1].y - fenceStruct[i].range.width, 1, vis_pub);
    else
      drawSquare(1, origin_pos[0].x + fenceStruct[i].range.height, origin_pos[0].y + fenceStruct[i].range.width, origin_pos[1].x - fenceStruct[i].range.height, origin_pos[1].y - fenceStruct[i].range.width, 2, vis_pub);
  }
  
}

/***************************************************************************************************************************************
* sound timer
* ***************************************************************************************************************************************/
void KevinFence::soundTimerCallback(const ros::TimerEvent &)
{
  // if(fenceFlag)
  robotSound_pub.publish(sound_msg);
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
  // std::cout << "===================" << std::endl; // debug print
  
  // print all point
  // std::cout << " mergedCloudSubCallback: ";
  // for (int i = 30; i < 50; i++)
  // {
  //   std::cout << msg.ranges[i] << "\t";
  // }
  // std::cout << std::endl;

  sensor_msgs::PointCloud out_pointcloud; // create a pointclound
  sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud); //pointclound2 to pointclound
  int pointCount = 0; // how many point
  for (int i = 0; i < out_pointcloud.points.size(); i++)
  {
    // std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << std::endl;
    for(int j = 0; j < fenceLevel; j++)
    {
      if ((out_pointcloud.points[i].x < (origin_pos[0].x + fenceStruct[i].range.height)) && (out_pointcloud.points[i].x > (origin_pos[1].x - fenceStruct[i].range.width))) // if x point inside the fence
      {
        if ((out_pointcloud.points[i].y < (origin_pos[0].y + fenceStruct[i].range.width)) && (out_pointcloud.points[i].y > (origin_pos[1].y - fenceStruct[i].range.width))) // if y point inside the fence
        {
          fenceStruct[i].pointCount++; // how many point ++
          // std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << std::endl;
        }
      }
      // std::cout << "pointCount: " << pointCount << std::endl;
    }
    
  }

  for(int i = fenceLevel-1; i >= 0; i--)
  {
    if (pointCount > fenceCount) // something inside fence
    {
      // std::cout << pointCount << std::endl; // debug print
      fenceStruct[i].flag = 1;
      soundTimer.start(); // timer start
      if (fenceStruct[i].lastFlag < fenceStruct[i].flag) // just at first time
      { 
        if(i == 0)
          soundTimer.setPeriod(ros::Duration(3));
        else if(i == 1)
          soundTimer.setPeriod(ros::Duration(1));
        robotSound_pub.publish(sound_msg);
      }
        
    }
    else
    {
      fenceStruct[i].flag = 0;
      soundTimer.stop(); // timer stop
    }
    fenceStruct[i].lastFlag = fenceStruct[i].flag; // record last flag
    }
}

/****************************************************************************************************************************************
* marker draw square
* ***************************************************************************************************************************************/
void KevinFence::drawSquare(int id, double x1, double y1, double x2, double y2, int color, ros::Publisher vis_pub)
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
  int qqX[5] = {0, 0, 1, 1, 0}; // square x
  int qqY[5] = {0, 1, 1, 0, 0}; // square y
  double posX[2] = {x1, x2};
  double posY[2] = {y1, y2};

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
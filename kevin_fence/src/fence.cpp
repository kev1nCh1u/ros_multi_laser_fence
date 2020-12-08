#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener1;
  tf::TransformListener listener2;

  ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  ros::Rate rate(10.0);

  while (node.ok())
  {
    /************************************
     * tf 
     * ***********************************/
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    try
    {
      ros::Time now = ros::Time::now();
      listener1.waitForTransform("/base_link", "/laser1",
                                now, ros::Duration(3.0));
      listener1.lookupTransform("/base_link", "/laser1",
                               ros::Time(0), transform1);

      listener2.waitForTransform("/base_link", "/laser2",
                                now, ros::Duration(3.0));
      listener2.lookupTransform("/base_link", "/laser2",
                               ros::Time(0), transform2);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // std::cout << " x1:" << transform1.getOrigin().x() << " y1:" << transform1.getOrigin().y() << " x2:" << transform2.getOrigin().x() << " y2:" << transform2.getOrigin().y() << std::endl;

    /************************************
     * marker
     * ***********************************/
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    geometry_msgs::Point p;


    p.x = transform1.getOrigin().x();
    p.y = transform1.getOrigin().y();
    marker.points.push_back(p);

    p.x = transform1.getOrigin().x();
    p.y = transform2.getOrigin().y();
    marker.points.push_back(p);

    p.x = transform2.getOrigin().x();
    p.y = transform2.getOrigin().y();
    marker.points.push_back(p);

    p.x = transform2.getOrigin().x();
    p.y = transform1.getOrigin().y();
    marker.points.push_back(p);

    p.x = transform1.getOrigin().x();
    p.y = transform1.getOrigin().y();
    marker.points.push_back(p);

    vis_pub.publish(marker);

    rate.sleep();
  }
  return 0;
};
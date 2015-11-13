#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "sensor_msgs/point_cloud_conversion.h"

class ScanToPointCloudTF 
{
public:
  ScanToPointCloudTF();
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
  
private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;

};

ScanToPointCloudTF::ScanToPointCloudTF()
{
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> (
    "/gazebo/laser_scan",
    20,
    &ScanToPointCloudTF::scan_callback,
    this);

  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (
    "/cloud",
    20,
    false);

  tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void ScanToPointCloudTF::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  try
  {
    sensor_msgs::PointCloud cloud;
    
    projector_.transformLaserScanToPointCloud(
      "ground_truth_pose",
      *scan,
      cloud,
      tfListener_);

    sensor_msgs::PointCloud2 cloud2;
    convertPointCloudToPointCloud2(cloud, cloud2);
    point_cloud_publisher_.publish(cloud2);
  }catch(...)
  {
    ROS_INFO_STREAM("old tf");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_point_cloud_tf");

  ScanToPointCloudTF scan_to_point_cloud_tf;
  ros::Rate r(30);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
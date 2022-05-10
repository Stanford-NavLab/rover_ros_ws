/*
 * Convert PointCloud2 to PCL and filter out NaNs
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class PCLFilter 
{

    private:
        ros::Subscriber sub;
        ros::Publisher pub;

    public:

        /**
        * Constructor
        */
        PCLFilter(ros::NodeHandle *nh) 
        {
            // Subscribe to incoming PointCloud2
            sub = nh->subscribe("velodyne_points", 1000, &PCLFilter::callback, this);

            // Publish PointCloud2 with new name
            pub = nh->advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, this);
        }

        /**
        * Point cloud callback
        */
        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
        {
            ROS_INFO("Received point cloud");

            //PointCloud::Ptr pcl_msg (new PointCloud);
            // pcl::PCLPointCloud2 pcl_msg;
            // pcl_conversions::toPCL(msg->data, pcl_msg)

            pub.publish(msg);
        }

};


/**
 * Main 
 */
int main (int argc, char **argv)
{
    ros::init(argc, argv, "pcl_filter");
    ros::NodeHandle nh;
    PCLFilter nc = PCLFilter(&nh);
    ros::spin();
}
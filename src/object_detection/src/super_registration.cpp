#include <ros/ros.h>
#include <pcl/registration/super4pcs.h>
#include <gr/algorithms/FunctorSuper4pcs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "super4pcs_registration_node");

    pcl::Super4PCS<pcl::PointNormal, pcl::PointNormal> align;

    ros::spin();

    return 0;
}
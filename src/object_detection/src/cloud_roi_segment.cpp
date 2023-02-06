
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection/DetectionsArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr organised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void cloud_cb(sensor_msgs::PointCloud2 input)
{
    pcl::fromROSMsg(input, *organised_cloud);
}

void roi_segment_cb(object_detection::DetectionsArray det_arr)
{
    // create the empty output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // iterate to find the detection with the greatest confidence
    object_detection::Detection best_det;
    best_det.confidence = 0;
    for (object_detection::Detection det : det_arr.objects)
    {
        if (det.confidence > best_det.confidence)
        {
            best_det = det;
        }
    }

    // iterate the points in the region of interest and add them to the output cloud
    for (int i=best_det.roi.y_offset; i<(best_det.roi.y_offset+best_det.roi.height); i++)
    {
        for (int j=best_det.roi.x_offset; j<(best_det.roi.x_offset+best_det.roi.width); j++)
        {
            output_cloud->push_back(organised_cloud->at(j,i));
        }
    }

    // publish output ros msg
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*output_cloud, ros_cloud);
    ros_cloud.header.frame_id = "camera_depth_optical_frame";
    pub.publish(ros_cloud);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cloud_roi_segment_node");
    ros::NodeHandle nh;
    
    ros::Subscriber cloud_sub = nh.subscribe("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber det_sub = nh.subscribe("inference_results", 1, roi_segment_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("topuc", 1);

    ros::spin();
}

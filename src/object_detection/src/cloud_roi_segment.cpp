
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
    std::cout << organised_cloud->height << " " << organised_cloud->width << std::endl;
    // iterate to find the most confident detection
    object_detection::Detection best_det;
    best_det.confidence = 0;
    for (object_detection::Detection det : det_arr.objects)
    {
        if (det.confidence > best_det.confidence)
        {
            best_det = det;
        }
    }

    organised_cloud += organised_cloud;

    for (int i=best_det.roi.y_offset; i<(best_det.roi.y_offset+best_det.roi.height); i++)
    {
        for (int j=best_det.roi.x_offset; j<(best_det.roi.x_offset+best_det.roi.width); j++)
        {
            std::cout << organised_cloud->at(j, i).x << " ";
        }
        std::cout << std::endl;
    }
    std::cout << best_det.class_id << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "hi";
    ros::init(argc, argv, "cloud_roi_segment_node");
    ros::NodeHandle nh;
    
    ros::Subscriber cloud_sub = nh.subscribe("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber det_sub = nh.subscribe("inference_results", 1, roi_segment_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("topuc", 1);

    ros::spin();
}

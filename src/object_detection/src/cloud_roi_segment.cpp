
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection/Detection2DArray.h>
#include <object_detection/Detection3D.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


ros::Publisher pub;
ros::Publisher cloud_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr organised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void cloud_cb(sensor_msgs::PointCloud2 input)
{
    pcl::fromROSMsg(input, *organised_cloud);
}

void roi_segment_cb(object_detection::Detection2DArray det_arr)
{
    // create the empty output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // iterate to find the detection with the greatest confidence
    object_detection::Detection2D best_det;
    best_det.confidence = 0;
    for (object_detection::Detection2D det : det_arr.objects)
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

    // create ros msg from cloud
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*output_cloud, ros_cloud);
    ros_cloud.header.frame_id = "camera_color_optical_frame";

    object_detection::Detection3D object;
    object.cloud = ros_cloud;
    object.detection2d = best_det;

    pub.publish(object);
    cloud_pub.publish(ros_cloud);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cloud_roi_segment_node");
    ros::NodeHandle nh;
    
    ros::Subscriber cloud_sub = nh.subscribe("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber det_sub = nh.subscribe("inference_results", 1, roi_segment_cb);

    pub = nh.advertise<object_detection::Detection3D>("detected_cloud", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);

    ros::spin();
}

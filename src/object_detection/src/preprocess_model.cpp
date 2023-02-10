#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

ros::Publisher pub;

typedef pcl::FieldComparison<pcl::PointXYZ> FieldComp;

pcl::PointCloud<pcl::PointXYZ>::Ptr read_obj(std::string filepath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadOBJFile<pcl::PointXYZ>(filepath, *object);
    return object;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_object(pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_rem;

    pcl::PassThrough<pcl::PointXYZ> pass;
    // keep everything except bottom
    pass.setInputCloud(object);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.003, 0.5);
    pass.filter(*object);

    // remove front wheels
    pcl::ConditionOr<pcl::PointXYZ>::Ptr front_wheel(new pcl::ConditionOr<pcl::PointXYZ>);
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::LT, 0)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::GT, 0.012)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::GT,  0.009)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::LT, -0.011)));
    cond_rem.setCondition(front_wheel);
    cond_rem.setInputCloud(object);
    cond_rem.filter(*object);

    // remove back wheels
    pcl::ConditionOr<pcl::PointXYZ>::Ptr back_wheel(new pcl::ConditionOr<pcl::PointXYZ>);
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::LT, 0)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::GT, 0.012)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::GT, -0.059)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::LT, -0.081)));
    cond_rem.setCondition(back_wheel);
    cond_rem.setInputCloud(object);
    cond_rem.filter(*object);

    return object;
}

pcl::PointCloud<pcl::PointNormal>::Ptr normal_est(pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_cloud(new pcl::PointCloud<pcl::PointNormal>);
    // Estimate normals for object and scene
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> nest;
    nest.setRadiusSearch(0.005);
    nest.setInputCloud(object);
    nest.compute(*normals_cloud);
    // concatenate the xyz fields and norm fields
    pcl::concatenateFields(*object, *normals_cloud, *normals_cloud);
    return normals_cloud;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_est(pcl::PointCloud<pcl::PointNormal>::Ptr object)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_cloud(new pcl::PointCloud<pcl::FPFHSignature33>);
    // Estimate features
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33 >  fest;
    fest.setRadiusSearch(0.01);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*feature_cloud);

    return feature_cloud;
}

void publish_object(pcl::PointCloud<pcl::PointNormal> object)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(object, output);
    output.header.frame_id = "map";
    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    std::string path = "/home/fif/lc252/inference-2d-3d/src/object_detection/model_geometry/";
    std::string model = "model_car_scaled";
    std::string filepath = path + model + std::string(".obj"); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("Loading Cloud");
    cloud = read_obj(filepath);

    ROS_INFO("Filtering");
    cloud = filter_object(cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr norm_cloud(new pcl::PointCloud<pcl::PointNormal>);
    ROS_INFO("Estimating Normals");
    norm_cloud = normal_est(cloud);
    pcl::io::savePCDFileASCII(path + model + std::string("_normal.pcd"), *norm_cloud);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat_cloud(new pcl::PointCloud<pcl::FPFHSignature33>);
    ROS_INFO("Estimating Features");
    feat_cloud = feature_est(norm_cloud);
    pcl::io::savePCDFileASCII(path + model + std::string("_features.pcd"), *feat_cloud);

    ROS_INFO("Publishing");
    while (ros::ok())
    {
        publish_object(*norm_cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
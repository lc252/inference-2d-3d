#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <object_detection/Detection3D.h>
#include <tf/transform_broadcaster.h>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

// Publishers
ros::Publisher object_aligned_pub;



void register_object_cb(object_detection::Detection3D detection)
{
    // Point clouds
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);

    ROS_INFO("Loading Clouds");
    // load scene
    pcl::fromROSMsg(detection.cloud, *scene);
    // load object
    // pcl::io::loadOBJFile<PointNT>("/home/fif/lc252/inference-2d-3d/src/object_detection/model_geometry/model_car_scaled.obj", *object);
    pcl::io::loadPCDFile<PointNT>("/home/fif/lc252/inference-2d-3d/src/object_detection/model_geometry/model_car_scaled_normal.pcd", *object);

    // Downsample
    pcl::VoxelGrid<PointNT> grid;
    grid.setLeafSize(0.005, 0.005, 0.005);
    grid.setInputCloud(object);
    grid.filter(*object);
    grid.setLeafSize(0.005, 0.005, 0.005);
    grid.setInputCloud(scene);
    grid.filter(*scene);

    // Segment and remove largest plane
    ROS_INFO("Segmenting");
    pcl::SACSegmentation<PointNT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.0025);   // tested previously
    seg.setInputCloud(scene);
    seg.segment(*inliers, *coeffs);
    if (inliers->indices.size() != 0)
    {
        pcl::ExtractIndices<PointNT> extractor;
        extractor.setInputCloud(scene);
        extractor.setIndices(inliers);
        extractor.setNegative(true);        // keep NON planar
        extractor.filter(*scene);
    }
    else
    {
        ROS_WARN("No plane found, no segmentation...");
    }

    // Estimate normals for object and scene
    ROS_INFO("Estimating Normals");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(object);
    nest.compute(*object);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // Estimate features
    ROS_INFO("Estimating Features Object");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    ROS_INFO("Estimating Features Scene");
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // Perform alignment
    ROS_INFO("Aligning");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(125000);               // Number of RANSAC iterations, increase to tradeoff speed for accuracy (50000)
    align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose, increase to tradeoff speed for accuracy (3)
    align.setCorrespondenceRandomness(5);            // Number of nearest features to use, increase to tradeoff speed for accuracy (5)
    align.setSimilarityThreshold(0.5f);              // Polygonal edge length similarity threshold, decrease to tradeoff speed for accuracy (0.9)
    align.setMaxCorrespondenceDistance(2.5f * 0.005); // Inlier threshold (2.5 * leaf size)
    align.setInlierFraction(0.7f);                  // Required inlier fraction for accepting a pose hypothesis, increase  (0.25)

    ROS_INFO("Attempting to register the model...");
    align.align(*object_aligned);

    if (!align.hasConverged())
    {
        ROS_WARN("Could not accurately register the model");
        return;
    }

    // get the transform Eigen
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    transformation = transformation.inverse();  // reverse from object->scene to scene->object
    Eigen::Vector3f t(transformation.block<3,1>(0,3));
    Eigen::Quaternionf q(transformation.block<3,3>(0,0));
    // create transform TF
    geometry_msgs::TransformStamped object_alignment_tf;
    object_alignment_tf.header.stamp = ros::Time::now();
    object_alignment_tf.header.frame_id = "camera_color_optical_frame";
    object_alignment_tf.child_frame_id = "aligned_object";
    object_alignment_tf.transform.translation.x = t.x();
    object_alignment_tf.transform.translation.y = t.y();
    object_alignment_tf.transform.translation.z = t.z();
    object_alignment_tf.transform.rotation.x = q.x();
    object_alignment_tf.transform.rotation.y = q.y();
    object_alignment_tf.transform.rotation.z = q.z();
    object_alignment_tf.transform.rotation.w = q.w();
    // broadcast
    static tf::TransformBroadcaster br;
    br.sendTransform(object_alignment_tf);

    // Create ros msg clouds
    sensor_msgs::PointCloud2 ros_object_aligned;
    pcl::toROSMsg(*object_aligned, ros_object_aligned);
    ros_object_aligned.header.frame_id = "camera_color_optical_frame";
    // Publish clouds
    object_aligned_pub.publish(ros_object_aligned);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    ros::NodeHandle nh;

    // setup pubs
    object_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("object_aligned", 1);

    // setup sub
    ros::Subscriber sub = nh.subscribe("detected_cloud", 1, register_object_cb);

    ros::spin();

    return (0);
}

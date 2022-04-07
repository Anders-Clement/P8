#include <ros/ros.h>
#include <ros/console.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;

#define VISUALIZE 1
// #define DOWNSAMPLE 0
// #define MAX_Z_DISTANCE 0.75

ros::Publisher pub;

std::tuple<float, float> calcRadius(pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0.05, 0.3);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);


  // initial guess for coeffiecients, 0 position, axis in Z direction. don't guess for radius
  coefficients_cylinder->values.push_back(0);
  coefficients_cylinder->values.push_back(0);
  coefficients_cylinder->values.push_back(0);
  coefficients_cylinder->values.push_back(0);
  coefficients_cylinder->values.push_back(0);
  coefficients_cylinder->values.push_back(1.0);

  // SACMODEL_CYLINDER - used to determine cylinder models. The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

// #if VISUALIZE==1
// 	extract.setInputCloud(cloud);
// 	extract.setIndices(inliers_cylinder);
// 	extract.setNegative(false);
// 	extract.filter(*cropped_cloud);

// 	for (int i = 0; i < cropped_cloud->size(); i++)
// 		cropped_cloud->at(i).g = 255;

// 	pcl::visualization::PCLVisualizer viewer("data viewer");

// 	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cloud_rgb(cloud);
// 	viewer.addPointCloud<PointT>(cloud, cloud_rgb, "sample cloud");
// 	//viewer.addPointCloud<PointT>(cloud, "sample cloud");
// 	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cropped_cloud_rgb(cropped_cloud);
// 	viewer.addPointCloud<PointT>(cropped_cloud, cropped_cloud_rgb, "cropped cloud");
// 	viewer.addCoordinateSystem(1.0);
// 	viewer.initCameraParameters();
// 	//coefficients_cylinder->values[0] = 0;
// 	//coefficients_cylinder->values[1] = 0;
// 	//coefficients_cylinder->values[2] = 0;
// 	//if (coefficients_cylinder->values[5] < 0)
// 	//	coefficients_cylinder->values[5] *= -1;
// 	viewer.addCylinder(*coefficients_cylinder, "cylinder");

// 	while (!viewer.wasStopped())
// 	{
// 		viewer.spinOnce(100);
// 		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
// 	}
// #endif

	float mse = 0;
	for(auto point: cloud.get()->points)
	{
		Eigen::Vector3f pi;
		pi[0] = point.x;
		pi[1] = point.y;
		pi[2] = point.z;
		Eigen::Vector3f pstar;
		pstar[0] = coefficients_cylinder->values[0];
		pstar[1] = coefficients_cylinder->values[1];
		pstar[2] = coefficients_cylinder->values[2];
		Eigen::Vector3f a;
		a[0] = coefficients_cylinder->values[3];
		a[1] = coefficients_cylinder->values[4];
		a[2] = coefficients_cylinder->values[5];

		Eigen::Vector3f pi_m_pstar = pi-pstar;
		float pi_m_pstar_dot_a = pi_m_pstar.dot(a);
		Eigen::Vector3f pi_m_pstar_dot_a_mult_a = pi_m_pstar_dot_a * a;
		Eigen::Vector3f vec = pi_m_pstar - pi_m_pstar_dot_a_mult_a;
		float norm = vec.norm();
		float deltaDist = norm - coefficients_cylinder->values[6];
		mse += deltaDist*deltaDist;
	}
	mse /= (float)cloud->size();

	return std::tuple<float,float> (abs(coefficients_cylinder->values[6]), mse);
}

// inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA> &in, pcl::PointCloud<pcl::PointXYZRGB> &out)
// {
//   out.width = in.width;
//   out.height = in.height;
//   out.points.resize(in.points.size());
//   for (size_t i = 0; i < in.points.size(); i++)
//   {
//     out.points[i].x = in.points[i].x;
//     out.points[i].y = in.points[i].y;
//     out.points[i].z = in.points[i].z;
//     out.points[i].r = in.points[i].r;
//     out.points[i].g = in.points[i].g;
//     out.points[i].b = in.points[i].b;
//   }
// }

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 cloud2;
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  //pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, cloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(cloud2,*cloudxyz);

  // pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // pcl::copyPointCloud(cloudxyz,cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

//   for（int nIndex = 0； nIndex < cloudxyz->points.size (); nIndex++）
// {
// 	pointCloud->points[nIndex].x;
// 	pointCloud->points[nIndex].y;
// 	pointCloud->points[nIndex].z;
// }
  
  for(auto point : cloudxyz->points)
  {
    std::cout << point << std::endl;
    pcl::PointXYZRGB rgbPoint;
    rgbPoint.x = point.x;
    rgbPoint.y = point.y;
    rgbPoint.z = point.z;
    cloud->push_back(rgbPoint);
  }


  // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);

  // ROS_INFO_STREAM("pre Hello ");

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  // pcl::fromPCLPointCloud2(*cloud2, temp_cloud);
  // PointCloudXYZRGBAtoXYZRGB(*temp_cloud, *cloud);
  // // pcl::PointCloud<PointT>::Ptr cloud3(cloud2);

  // ROS_DEBUG("check2 %s", "World");

  calcRadius(cloud);

  // ROS_INFO_STREAM("post Hello ");
  // // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // // Publish the data
  // pub.publish(output);
  // ROS_INFO_STREAM("post pub ");

}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_ransac"); //"my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
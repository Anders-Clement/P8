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
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

typedef pcl::PointXYZRGB PointT;

#define VISUALIZE 1
// #define DOWNSAMPLE 0
// #define MAX_Z_DISTANCE 0.75

ros::Publisher pub;
ros::Publisher vis_pub;

// projection of vector v1 onto v2
Eigen::Vector3f projection( Eigen::Vector3f v1, Eigen::Vector3f v2 )
{
  float v2_ls = v2.squaredNorm();
  float dotpro = v2.dot(v1);
  return v2 * (dotpro/v2_ls);

}


std::tuple<Eigen::Vector3f, Eigen::Vector3f> calcRadius(pcl::PointCloud<PointT>::Ptr cloud)
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
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  //seg.setRadiusLimits(0.05, 0.3);
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
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(*cropped_cloud);

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

  Eigen::Vector3f com;

  int n = 0;

	for(auto point: cropped_cloud.get()->points)
	{
    com[0] += point.x;
    com[1] += point.y;
    com[2] += point.z;
    n += 1;

		// Eigen::Vector3f pi;
		// pi[0] = point.x;
		// pi[1] = point.y;
		// pi[2] = point.z;
		// Eigen::Vector3f pstar;
		// pstar[0] = coefficients_cylinder->values[0];
		// pstar[1] = coefficients_cylinder->values[1];
		// pstar[2] = coefficients_cylinder->values[2];
		// Eigen::Vector3f a;
		// a[0] = coefficients_cylinder->values[3];
		// a[1] = coefficients_cylinder->values[4];
		// a[2] = coefficients_cylinder->values[5];

		// Eigen::Vector3f pi_m_pstar = pi-pstar;
		// float pi_m_pstar_dot_a = pi_m_pstar.dot(a);
		// Eigen::Vector3f pi_m_pstar_dot_a_mult_a = pi_m_pstar_dot_a * a;
		// Eigen::Vector3f vec = pi_m_pstar - pi_m_pstar_dot_a_mult_a;
		// float norm = vec.norm();
		// float deltaDist = norm - coefficients_cylinder->values[6];
		// mse += deltaDist*deltaDist;
	}
	// mse /= (float)cloud->size();
  com /= n;

  Eigen::Vector3f pstar;
  pstar[0] = coefficients_cylinder->values[0];
  pstar[1] = coefficients_cylinder->values[1];
  pstar[2] = coefficients_cylinder->values[2];
  Eigen::Vector3f a;
  a[0] = coefficients_cylinder->values[3];
  a[1] = coefficients_cylinder->values[4];
  a[2] = coefficients_cylinder->values[5];
  
  
  Eigen::Vector3f proj_com = projection(com-pstar,a);

  return std::tuple<Eigen::Vector3f,Eigen::Vector3f>(proj_com+pstar,a);

	// return std::tuple<float,float> (abs(coefficients_cylinder->values[6]), [average_x,average_y,average_y]);
}


void vis_marker(geometry_msgs::PoseStamped pose)
{
  visualization_msgs::Marker marker;
  marker.header = pose.header;
  marker.pose = pose.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.action = 0;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(10);
  marker.type = 0;

  vis_pub.publish(marker);

}



void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 cloud2;
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  // pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, cloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(cloud2,*cloudxyz);

  // pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // pcl::copyPointCloud(cloudxyz,cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//   for（int nIndex = 0； nIndex < cloudxyz->points.size (); nIndex++）
// {
// 	pointCloud->points[nIndex].x;
// 	pointCloud->points[nIndex].y;
// 	pointCloud->points[nIndex].z;
// }
  
  for(auto point : cloudxyz->points)
  {
    pcl::PointXYZRGB rgbPoint;
    rgbPoint.x = point.x;
    rgbPoint.y = point.y;
    rgbPoint.z = point.z;
    if (rgbPoint.x != rgbPoint.x || rgbPoint.y != rgbPoint.y || rgbPoint.z != rgbPoint.z)
    {
      continue;
    }

    cloud->push_back(rgbPoint);

  }

  if (cloud->size() < 10)
  {
    return;
  }

  // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (*cloud);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);



  std::tuple<Eigen::Vector3f,Eigen::Vector3f>cyl_vecs;

  try
  {
      // code that could cause exception
      cyl_vecs = calcRadius(cloud);
  }
  catch (const std::exception &exc)
  {
      // catch anything thrown within try block that derives from std::exception
      std::cerr << exc.what();
  }

  if (isinf(std::get<0>(cyl_vecs)[0])) {
    return;
  }

  
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = cloud_msg->header;
  pose_msg.pose.position.x = std::get<0>(cyl_vecs)[0];
  pose_msg.pose.position.y = std::get<0>(cyl_vecs)[1];
  pose_msg.pose.position.z = std::get<0>(cyl_vecs)[2];
  pose_msg.pose.orientation.x = std::get<1>(cyl_vecs)[0];
  pose_msg.pose.orientation.y = std::get<1>(cyl_vecs)[1];
  pose_msg.pose.orientation.z = std::get<1>(cyl_vecs)[2];
  pose_msg.pose.orientation.w = 0;



  pub.publish(pose_msg);


  vis_marker(pose_msg);

  // // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // // Publish the data
  // 
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
  pub = nh.advertise<geometry_msgs::PoseStamped>("detection", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>("vis_marker",1);

  // Spin
  ros::spin();
}
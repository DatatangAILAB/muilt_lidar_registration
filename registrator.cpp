#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h> 
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>
#include "json/json.h"
#include <iostream>
#include <fstream>
#include <string>

struct PointXYZIL {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  unsigned int label = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity,intensity)
    (unsigned int, label, label)
    )



void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " pre.pcd current.pcd pre.json current.json" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

void rangeFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,float min_scan_range,float max_scan_range,float min_z_range,float max_z_range ) {
  for (int i=0;i<cloud_in->points.size();i++) {
    if (!isnan(cloud_in->points[i].x)) {
      float r=sqrt(pow(cloud_in->points[i].x, 2.0) + pow(cloud_in->points[i].y, 2.0));
      if (min_scan_range < r && r < max_scan_range && min_z_range < cloud_in->points[i].z && cloud_in->points[i].z < max_z_range)
      {
        cloud_out->push_back(cloud_in->points[i]);
      }
    }
  }
  return;
}

std::vector<float> matrix2angle(Eigen::Matrix4f rotateMatrix)
{
  float sy = (float)sqrt(rotateMatrix(0,0) * rotateMatrix(0,0) + rotateMatrix(1,0)*rotateMatrix(1,0));
  bool singular = sy < 1e-6; // If
  float x, y, z;
  if (!singular)
  {
    x = (float)atan2(rotateMatrix(2,1), rotateMatrix(2,2));
    y = (float)atan2(-rotateMatrix(2,0), sy);
    z = (float)atan2(rotateMatrix(1, 0), rotateMatrix(0, 0));
  }
  else
  {
    x = (float)atan2(-rotateMatrix(1, 2), rotateMatrix(1, 1));
    y = (float)atan2(-rotateMatrix(2, 0), sy);
    z = 0;
  }
  std::vector<float> i;
  i.push_back(x);
  i.push_back(y);
  i.push_back(z);
  return i;
}


Eigen::Matrix4f euler2Rotation( Eigen::Vector3f  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);
 
    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);
 
    Eigen::Matrix4f RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp, 0.06,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr, 0,
            -sp,         cp*sr,           cp*cr,0,
            0,0,0,1;
    return RIb;
}

pcl::console::TicToc timecal;

int main(int argc, char **argv){
    // Show help
    if (argc!=2) {
      showHelp (argv[0]);
      return 0;
    }

  // 加载前一帧参考文件
  pcl::PointCloud<PointXYZIL>::Ptr cloud (new pcl::PointCloud<PointXYZIL> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
    showHelp (argv[0]);
    return -1;
  }
  std::cout << argv[1] << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3 (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4 (new pcl::PointCloud<pcl::PointXYZI> ());

  for (int i=0;i<cloud->points.size();i++) {
    pcl::PointXYZI point;
    point.x=cloud->points[i].x;
    point.y=cloud->points[i].y;
    point.z=cloud->points[i].z;
    point.intensity=cloud->points[i].intensity;

    if (cloud->points[i].label==1||cloud->points[i].label==2||cloud->points[i].label==4)
        cloud_1->push_back(point);
    if (cloud->points[i].label==2)
        cloud_2->push_back(point);
    if (cloud->points[i].label==3)
        cloud_3->push_back(point);
    if (cloud->points[i].label==4)
        cloud_4->push_back(point);
    
  }
  
  const double min_scan_range = 5.0;
  const double max_scan_range = 150.0;
  const double min_z_range = -0.1;
  const double max_z_range = 5;

  //过滤稀疏点
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_rangefilter (new pcl::PointCloud<pcl::PointXYZI> );
  rangeFilter(cloud_1, cloud1_rangefilter, min_scan_range, max_scan_range,min_z_range,max_z_range );

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3_rangefilter (new pcl::PointCloud<pcl::PointXYZI> );
  rangeFilter(cloud_3, cloud3_rangefilter, min_scan_range, max_scan_range,min_z_range,max_z_range );


  //精配准
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr post_icp_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  

  icp.setInputSource(cloud3_rangefilter);
  icp.setInputTarget(cloud1_rangefilter);

  icp.setMaxCorrespondenceDistance(0.1);
  icp.setMaximumIterations (1000); //for test 300
  
  timecal.tic();
  icp.align(*post_icp_cloud);  
  std::cout<<"Finished ICP Regisration in "<<int(timecal.toc()/1000)<<"s"<<std::endl;
  std::cout <<"score: " << icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f icp_transform=icp.getFinalTransformation();
  std::cout << "icp_transform:"<<std::endl << icp_transform << std::endl;

  std::vector<float> angle=matrix2angle(icp_transform);
  std::cout<<"rotation angle:"<<angle[0]<<"\t"<<angle[1]<<"\t"<<angle[2]<<std::endl;

  
  
  /*
  Eigen::Vector3f  eulerAngles; 
  eulerAngles << 0,0,0.0386166;
  Eigen::Matrix4f icp_transform =euler2Rotation(eulerAngles);
  std::cout << icp_transform << std::endl;
  */

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_3, *transformed_cloud, icp_transform);
  
  
  //可视化的代码
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_1_handler (cloud_1, 255, 255, 255);  //红-当前 
  viewer.addPointCloud (cloud_1, cloud_1_handler, "cloud_1");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_handler (transformed_cloud, 255, 0, 0);  //红-当前 
  viewer.addPointCloud (transformed_cloud, transformed_cloud_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  
  

}


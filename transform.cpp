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

int main(int argc, char **argv){
    // Show help
    if (argc!=3) {
      showHelp (argv[0]);
      return 0;
    }

  //加载文件
  pcl::PointCloud<PointXYZIL>::Ptr cloud (new pcl::PointCloud<PointXYZIL> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
    showHelp (argv[0]);
    return -1;
  }
  std::cout << argv[1] << std::endl;

  //分类成4个雷达，其中1、2、4没有问题作为一组cloud1，3作为一组cloud3
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
  
  
  //经过配准后获得的变换矩阵
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
 
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  transform_1 (0,0) =  0.999617 ;
  transform_1 (0,1) = -0.0273586 ;
  transform_1 (0,2) = -0.00818321  ;

  transform_1 (1,0) = 0.0272533   ;
  transform_1 (1,1) = 0.999565 ;
  transform_1 (1,2) = -0.0124118  ;

  transform_1 (2,0) =  0.00851969  ;
  transform_1 (2,1) = 0.0121844 ;
  transform_1 (2,2) = 0.999934   ;

  // Define a translation of 2.5 meters on the x axis.
  transform_1 (0,3) = 0.0652719   ;
  transform_1 (1,3) = -0.000150033  ;
  transform_1 (2,3) = 0.0561919  ;
 
  // Print the transformation
  std::cout << transform_1 << std::endl;
  
  //将cloud3配准到cloud1
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_3, *transformed_cloud, transform_1);
  
  //添加进cloud1
  *cloud_1+=*transformed_cloud;

  //保存
  pcl::PCDWriter writer;
  writer.write(argv[2],*cloud_1,true);

  std::cout << "Write  "
    << cloud_1->points.size()
    << " data points into "<<argv[2]
    << std::endl;
  
  

}


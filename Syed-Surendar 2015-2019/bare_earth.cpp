#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);
  std::string input_file = " ";
  std::cout << "Enter the name of the PCD File:";
  std::cin >> input_file;

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB>(input_file, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;

  
  writer.write<pcl::PointXYZRGB>(input_file+"vaighien.pcd", *cloud_filtered, false);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZRGB>(input_file+"vaighien.pcd", *cloud_filtered, false);

  return (0);
}
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/irange.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/common/transforms.h>   


#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "laser_geometry/laser_geometry.h"

#include <dirent.h>

#include <fstream>

#include "laser_people_detect/calc_people_features.h"

using namespace std;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_POS, LOADING_NEG, LOADING_MIX};

class PclSegment
{
public:

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;

  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float mean_k_;
  float std_dev_mult_thresh_;
  float leaf_size_x_;
  float leaf_size_y_;
  float leaf_size_z_;
  float cluster_tolerance_;
  float min_cluster_size_;
  float max_cluster_size_;

  PclSegment(vector<float> param_vec)
  {
        min_x_              = param_vec[0];
        max_x_              = param_vec[1];
        min_y_              = param_vec[2];
        max_y_              = param_vec[3];
        mean_k_             = param_vec[4];
        std_dev_mult_thresh_ = param_vec[5];
        leaf_size_x_        = param_vec[6];
        leaf_size_y_        = param_vec[7];
        leaf_size_z_        = param_vec[8];
        cluster_tolerance_  = param_vec[9];
        min_cluster_size_   = param_vec[10];
        max_cluster_size_   = param_vec[11];
  }

  void loadData(const char* folder)
  {
      // features wll saved to pos_data_ neg_data_
      struct dirent *ptr;
      DIR *dir;
      dir = opendir(folder);
      vector<string> files;
      cout << "opening folder: " << folder << "..." << endl;
      while((ptr=readdir(dir))!=NULL)
      {
          // skip "." and ".." file
          if(ptr->d_name[0] == '.')
              continue;
          string cc = string(folder) + string(ptr->d_name); 
          //files.push_back(const_cast<char*>(cc.c_str()));
          files.push_back(cc);
      }
      closedir(dir);

      for (int i = 0; i < files.size(); ++i)
      {
          cout << "Loading data from file: "<< files[i] << endl;
          loadCb(files[i], pos_data_);
      }
  }

  void loadCb(string const& rosbag_file, vector< vector<float> > &data)
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    rosbag::View view(bag); 

    int message_num = 0;
    int initial_data_size = (int)data.size();

    // ***********************start from here********************
    std::vector<sensor_msgs::LaserScan> bundle;
        
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;
    // use boost to get each scan
    BOOST_FOREACH(rosbag::MessageInstance m, view)
    {
        sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
        if (scan != NULL)
        {
            bundle.push_back(*scan);
            //**************merge these scans and changge to pcl
            cloud.points.clear();
            cloud.channels.clear();

            vector<sensor_msgs::ChannelFloat32> channel(1);
            cloud.channels = channel;
            projector_.projectLaser(*scan, cloud);

            // PointCloud to PointCloud2
            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2 (cloud, pc2);
                
            // PointCloud2 to pcl::PointXYZ
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(pc2, *pcl_cloud);

            //**************pcl filter***************
            //filtering the cloud by using statistical removal
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            // if(pcl_cloud->points.size() > 10)
            // {
            //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;                          
            //     sor.setInputCloud(pcl_cloud);                                                   
            //     sor.setMeanK(mean_k_);                                                           
            //     sor.setStddevMulThresh(std_dev_mult_thresh_);                                                
            //     sor.filter(*pcl_cloud);   

            //     // voxel
            //     pcl::VoxelGrid<pcl::PointXYZ> sor1;                                         
            //     sor1.setInputCloud(pcl_cloud);                                                  
            //     sor1.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);                                      
            //     sor1.filter(*cloud_filtered);  
            // }

            //*************pcl cluster**************
            std::vector<pcl::PointIndices> cluster_indices;
            if(pcl_cloud->points.size() > 10)
            {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud (pcl_cloud);  

                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
                ece.setClusterTolerance(cluster_tolerance_); //15cm, the unit is meter
                ece.setMinClusterSize(min_cluster_size_); //the minimun number of points to form a cluster is 15
                ece.setMaxClusterSize(max_cluster_size_); //the maximum number of points in a cluster
                ece.setSearchMethod(tree);
                ece.setInputCloud(pcl_cloud);
                ece.extract(cluster_indices);
            }

            //*************extract and save cluster ****************
            int ind = 0;
            for (vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
                float c_x = 0.0;
                float c_y = 0.0;
                for (vector<int>::const_iterator j=it->indices.begin(); j!=it->indices.end(); ++j)
                {
                    cluster->points.push_back(pcl_cloud->points[(int)(*j)]);
                    c_x += pcl_cloud->points[*j].x;
                    c_y += pcl_cloud->points[*j].y;
                }
                c_x /= cluster->points.size();
                c_y /= cluster->points.size();

                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = true; //no invalid points        
                for (pcl::PointCloud<pcl::PointXYZ>::iterator j=cluster->begin(); j!=cluster->end(); ++j)
                {
                    j->x = j->x-c_x;
                    j->y = j->y-c_y;
                }

                // save pcl to pcd file
                string file_name = rosbag_file.substr(0, rosbag_file.find("."));
                file_name += "_"+to_string(message_num)+"_"+to_string(ind)+".pcd";
                pcl::io::savePCDFile(file_name, *cluster);
                cout <<  "saving to file: " << file_name <<endl;
                    
                cluster->clear();
                ind ++;
            }
 
            message_num++;
         }
    }
    bag.close();

    cout << "\t Got " << message_num << " scan messages, "<< (int)data.size() - initial_data_size << "  samples" << endl;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_body_detector");
  ros::NodeHandle nh;

  vector<float> param_vec(12, 0);
  if(!ros::param::get("~min_x", param_vec[0]))
      param_vec[0] = 0.0;
  if(!ros::param::get("~max_x", param_vec[1]))
      param_vec[1] = 10.0;
  if(ros::param::get("~min_y", param_vec[2]))
      param_vec[2] = -2.0;
  if(ros::param::get("~max_y", param_vec[3]))
      param_vec[3] = 2.0;
  if(!ros::param::get("~mean_k", param_vec[4]))
      param_vec[4] = 10.0;
  if(!ros::param::get("~std_dev_mult_thresh", param_vec[5]))
      param_vec[5] = 1.0;
  if(!ros::param::get("~leaf_size_x", param_vec[6]))
      param_vec[6] = 0.02;
  if(!ros::param::get("~leaf_size_y", param_vec[7]))
      param_vec[7] = 0.02;
  if(!ros::param::get("~leaf_size_z", param_vec[8]))
      param_vec[8] = 0.02;
  if(!ros::param::get("~cluster_tolerance", param_vec[9]))
      param_vec[9] = 0.2;
  if(!ros::param::get("~min_cluster_size", param_vec[10]))
      param_vec[10] = 20.0;
  if(!ros::param::get("~max_cluster_size", param_vec[11]))
      param_vec[11] = 2000.0;
  if(!ros::param::get("~flip_left_right", param_vec[12]))
      param_vec[12] = 0;

  PclSegment ps(param_vec);

  cout << "Loading data..." << endl;
  for (int i = 1; i < argc; i++)
  {
      ps.loadData(argv[i]);
  }

  //ros::spin();
  return 0;
}

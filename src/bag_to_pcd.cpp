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
#include <pcl/common/transforms.h>   
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "laser_geometry/laser_geometry.h"

#include <dirent.h>
#include <fstream>

using namespace std;
using namespace ros;

void save_pcd(string const& rosbag_file)
{
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    rosbag::View view(bag); 

    int message_num = 0;
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

            // save pcl to pcd file
            string file_name = rosbag_file.substr(0, rosbag_file.find("."));
            file_name += "_"+to_string(message_num)+".pcd";
            pcl::io::savePCDFile(file_name, *pcl_cloud);
            cout <<  "saving to file: " << file_name <<endl;
                    
            pcl_cloud->clear();
            message_num++;
         }
    }
    bag.close();

    cout << "\t Got " << message_num << " scan messages, "<< endl;
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
      save_pcd(files[i]);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_body_detector");
  ros::NodeHandle nh;
  cout << "Loading data..." << endl;
  for (int i = 1; i < argc; i++)
  {
      loadData(argv[i]);
  }

  //ros::spin();
  return 0;
}

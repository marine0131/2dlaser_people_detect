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
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/irange.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>   


#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

#include <laser_people_detect/calc_people_features.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "laser_geometry/laser_geometry.h"

using namespace std;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_POS, LOADING_NEG, LOADING_MIX, LOADING_TEST};

class TrainPeopleDetector
{
public:

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;
  vector< vector<float> > test_data_;

  CvRTrees forest;

  float connected_thresh_;
  float min_size_;
  int feat_count_;
  vector<float> pos_region_;

  TrainPeopleDetector(float min_size, float connect_th, vector<float> pos_region) : min_size_(20), connected_thresh_(0.06), feat_count_(0)
  {
      connected_thresh_ = connect_th;
      min_size_ = min_size;
      pos_region_ = pos_region;
  }

  void loadData(LoadType load, char* file)
  {
    if (load != LOADING_NONE)
    {
      // features wll saved to pos_data_ neg_data_
      switch (load)
      {
          case LOADING_POS:
              cout << "Loading positive training data from file: "<< file << endl;
              loadCb(file, pos_data_);
              break;
          case LOADING_NEG:
              cout << "Loading negative training data from file: "<< file << endl;
              loadCb(file, neg_data_);
              break;
          case LOADING_MIX:
              cout << "Loading mix training data from file: "<< file << endl;
              loadCb(true, file, pos_data_, neg_data_);
              break;
          case LOADING_TEST:
              cout << "Loading test data from file: "<< file << endl;
              loadCb(file, test_data_);
              break;
          default:
              break;
      }
    }
  }
  void loadCb(const char* rosbag_file, vector< vector<float> > &data)
  {
      loadCb(false, rosbag_file, data, data);
  }
  void loadCb(bool is_mix, const char* rosbag_file, vector< vector<float> > &data, vector< vector<float> > &neg)
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    rosbag::View view(bag); 

    int message_num = 0;
    int initial_data_size = (int)data.size();
    int initial_neg_data_size = (int)neg.size();

    // ***********************start from here********************
    int bundle_num = 3; // every 3 scan bundle together
    std::vector<sensor_msgs::LaserScan> bundle;
        
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud merge_cloud;
    // use boost to get each scan
    BOOST_FOREACH(rosbag::MessageInstance m, view)
    {
        sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
        if (scan != NULL)
        {
            bundle.push_back(*scan);
            if (bundle.size() >= bundle_num)
            {
                //**************merge these scans and changge to pcl
                merge_cloud.points.clear();
                merge_cloud.channels.clear();

                float idx_inc = 0.0; 
                vector<sensor_msgs::ChannelFloat32> channel(1);
                merge_cloud.channels = channel;
                for (std::vector<sensor_msgs::LaserScan>::const_iterator i=bundle.begin(); i!=bundle.end(); ++i)
                {
                    projector_.projectLaser(*i, cloud);
                    boost::push_back(merge_cloud.points, cloud.points);
                    merge_cloud.channels[0].name = cloud.channels[1].name;
                    boost::push_back(merge_cloud.channels[0].values, boost::irange(idx_inc, idx_inc+cloud.points.size()));                    
                    idx_inc += cloud.points.size();
                }
                
                // PointCloud to PointCloud2
                sensor_msgs::PointCloud2 pc2;
                sensor_msgs::convertPointCloudToPointCloud2 (merge_cloud, pc2);
                
                // PointCloud2 to pcl::PointXYZ
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(pc2, *pcl_cloud);

                //**************pcl filter***************
                //filtering the cloud by using statistical removal                          
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;                          
                sor.setInputCloud(pcl_cloud);                                                   
                sor.setMeanK(10);                                                           
                sor.setStddevMulThresh(1.0);                                                
                sor.filter(*pcl_cloud);   

                // voxel
                pcl::VoxelGrid<pcl::PointXYZ> sor1;                                         
                sor1.setInputCloud(pcl_cloud);                                                  
                sor1.setLeafSize(0.02f, 0.02f, 0.02f);                                      
                sor1.filter(*cloud_filtered);  

                //*************pcl cluster**************
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud (cloud_filtered);  

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
                ece.setClusterTolerance(0.15); //15cm, the unit is meter
                ece.setMinClusterSize(15); //the minimun number of points to form a cluster is 15
                ece.setMaxClusterSize(25000); //the maximum number of points in a cluster
                ece.setSearchMethod(tree);
                ece.setInputCloud(cloud_filtered);
                ece.extract(cluster_indices);

                //*************calculate features****************
                for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
                {    
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
                    float c_x = 0.0;
                    float c_y = 0.0;
                    for (std::vector<int>::const_iterator j=it->indices.begin(); j!=it->indices.end(); ++j)
                    {                                                                       
                        cluster->points.push_back(cloud_filtered->points[(int)(*j)]);                 
                        c_x += cloud_filtered->points[*j].x;
                        c_y += cloud_filtered->points[*j].y;
                    }                                                                       
                    cluster->width = cloud_filtered->points.size();                    
                    cluster->height = 1;                                              
                    cluster->is_dense = true; //no invalid points        
                
                    c_x /= cluster->points.size();
                    c_y /= cluster->points.size();
                    if(is_mix)
                    {
                        if(c_x > pos_region_[0] && c_x < pos_region_[1] && c_y > pos_region_[2] && c_y < pos_region_[3])
                            data.push_back(calcPeopleFeatures(cluster));                 
                        //else
                        //    neg.push_back(calcPeopleFeatures(cluster));
                        continue;
                    }
                    data.push_back(calcPeopleFeatures(cluster));                 
                }
                bundle.clear();
            }
 
            message_num++;
         } 
    }
    bag.close();

    if(is_mix)
    {
        cout << "\t Got " << message_num << " scan messages, "<< (int)data.size() - initial_data_size << "  pos samples, from " << rosbag_file << endl;
        // cout << "\t Got " << message_num << " scan messages, "<< (int)neg.size() - initial_neg_data_size << "  neg samples, from " << rosbag_file << endl;
        return;
    }
    cout << "\t Got " << message_num << " scan messages, "<< (int)data.size() - initial_data_size << "  samples, from " << rosbag_file << endl;
  } 


  void train()
  {
    int sample_size = pos_data_.size() + neg_data_.size(); //sample count
    feat_count_ = pos_data_[0].size();  //feature count

    CvMat* cv_data = cvCreateMat(sample_size, feat_count_, CV_32FC1);  //cv data array
    CvMat* cv_resp = cvCreateMat(sample_size, 1, CV_32S);  //cv result array

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat(1, feat_count_ + 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D(var_type, feat_count_, CV_VAR_CATEGORICAL);

    float priors[] = {1.0, 1.0};

    CvRTParams fparam(8, 20, 0, false, 10, priors, false, 5, 50, 0.001f, CV_TERMCRIT_ITER);
    fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);

    forest.train(cv_data, CV_ROW_SAMPLE, cv_resp, 0, 0, var_type, 0, fparam);

    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  void test()
  {
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    int pos_right = 0;
    int pos_total = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin(); i != pos_data_.end(); i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        pos_right++;
      pos_total++;
    }

    int neg_right = 0;
    int neg_total = 0;
    for (vector< vector<float> >::iterator i = neg_data_.begin(); i != neg_data_.end(); i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) < 0)
        neg_right++;
      neg_total++;
    }

    int test_right = 0;
    int test_total = 0;
    for (vector< vector<float> >::iterator i = test_data_.begin(); i != test_data_.end(); i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        test_right++;
      test_total++;
    }

    cout << "Pos train set: \t" << pos_right << "/" << pos_total << "\t" << (float)(pos_right) / pos_total * 100 << "%" << endl;
    cout << "Neg train set: \t" << neg_right << "/" << neg_total << "\t" << (float)(neg_right) / neg_total * 100 <<"%" << endl;
    cout << "Test set: \t" << test_right << "/" << test_total << "\t" << (float)(test_right) / test_total * 100 << "%" << endl;

    cvReleaseMat(&tmp_mat);

  }

  void save(char* file)
  {
    forest.save(file);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_body_detector");
  ros::NodeHandle nh;

  float connect_th = 0.06; 
  float min_size = 20; 
  vector<float> pos_region(4, 0);
  ros::param::get("~connect_th", connect_th);
  ros::param::get("~min_size", min_size);
  ros::param::get("~min_x", pos_region[0]);
  ros::param::get("~max_x", pos_region[1]);
  ros::param::get("~min_y", pos_region[2]);
  ros::param::get("~max_y", pos_region[3]);
  cout << "connect_th: " << connect_th << " min_size: " << min_size << endl;

  TrainPeopleDetector tpd(min_size, connect_th, pos_region);

  LoadType loading = LOADING_NONE;

  char save_file[200];
  save_file[0] = 0;

  cout << "Loading data..." << endl;
  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i], "--pos"))
      loading = LOADING_POS;
    else if (!strcmp(argv[i], "--neg"))
      loading = LOADING_NEG;
    else if (!strcmp(argv[i], "--mix"))
        loading = LOADING_MIX;
    else if (!strcmp(argv[i], "--test"))
      loading = LOADING_TEST;
    else if (!strcmp(argv[i], "--save"))
    {
      if (++i < argc)
        strncpy(save_file, argv[i], 100);
      continue;
    }
    else
      tpd.loadData(loading, argv[i]);
  }

  cout << "Training classifier..." << endl;
  tpd.train();

  cout <<"Evlauating classifier..."<< endl;
  tpd.test();

  if (strlen(save_file) > 0)
  {
    cout << "Saving classifier as: "<< save_file << endl;
    tpd.save(save_file);
  }

  //ros::spin();
  return 0;
}

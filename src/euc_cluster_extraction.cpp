#include <ros/ros.h>
#include <vector>

#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/cxcore.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <laser_people_detect/calc_people_features.h>

using namespace std;
bool MARKER_PUB = false;
bool FILTERED_CLOUD_PUB = false;

class LaserProcessor
{
protected:
    ros::NodeHandle nh_;

public:
    CvRTrees forest;
    int feat_count_;

    ros::Publisher marker_pub;
    ros::Publisher cloud_pub;
    ros::Publisher people_pub;
    ros::Subscriber cloud_sub;

    LaserProcessor(ros::NodeHandle nh, string forest_file): nh_(nh)
    {
        forest.load((char*)forest_file.data());
        feat_count_ = forest.get_active_var_mask()->cols;
        cout << "Loaded forest with " << feat_count_ << " features: " << forest_file << endl;

        cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud_raw", 1, &LaserProcessor::cloud_cb, this);
        
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray> ("/cluster_center", 1);
        people_pub = nh_.advertise<sensor_msgs::PointCloud> ("/people_detected", 1);
    }

    ~LaserProcessor()
    {
    }


    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        cout << "callback" << endl;
        //ros::Time begin = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // voxel grid
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor1;
        sor1.setInputCloud(cloud);
        sor1.setLeafSize(0.02f, 0.02f, 0.02f);
        sor1.filter(*cloud_sor1);

        //filtering the cloud by using statistical removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud_sor1);
        sor2.setMeanK(10);
        sor2.setStddevMulThresh(1.0);
        sor2.filter(*cloud);


        if(FILTERED_CLOUD_PUB)
        {
            sensor_msgs::PointCloud2 cloud_filtered;
            pcl::toROSMsg(*cloud, cloud_filtered);
            cloud_pub.publish(cloud_filtered);
        }
        
        //ros::Duration end = ros::Time::now() - begin;
        //std::cout << "filter_time: " << end << std::endl;
        //begin = ros::Time::now();

        //creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        //extract the points into clusters, vector 'cluster_indices' stores the indices of all the clusters
        //for example, cluster_indices[0] contains the indices of the points of the first cluster
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setClusterTolerance(0.15); //15cm, the unit is meter
        ece.setMinClusterSize(50); //the minimun number of points to form a cluster is 200
        ece.setMaxClusterSize(25000); //the maximum number of points in a cluster
        ece.setSearchMethod(tree);
        ece.setInputCloud(cloud);
        ece.extract(cluster_indices);

        //end = ros::Time::now() - begin;
        //std::cout << "extract_time" << end<< std::endl;
        //begin = ros::Time::now();

        //seperate the clusters and store them into a vector 'extracted_clusters'
        std::vector<sensor_msgs::PointCloud2> extracted_clusters;
        // output PointCloud center coordinate of every cluster & probability
        sensor_msgs::PointCloud cluster_center;
        sensor_msgs::ChannelFloat32 probability_channel;
        probability_channel.name = "probability";
        cluster_center.channels.push_back(probability_channel);

        for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            geometry_msgs::Point32 center_point;
            for (std::vector<int>::const_iterator j=it->indices.begin(); j!=it->indices.end(); ++j)
            {   
                cloud_cluster->points.push_back(cloud->points[*j]);
                center_point.x += cloud->points[*j].x;
                center_point.y += cloud->points[*j].y;
            }
            cloud_cluster->width = cloud_cluster->points.size();
            //ROS_INFO("point_size %d", cloud_cluster->points.size());
            cloud_cluster->height = 1;

            //calc cluster center
            center_point.x /= it->indices.size();
            center_point.y /= it->indices.size();
            cluster_center.points.push_back(center_point);

            //*****************calculate features*********************
            std::vector<float> feature_data;
            feature_data = calcPeopleFeatures(cloud_cluster);

            //change vector to cvMat format
            CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);   
            for (int k = 0; k < feat_count_; k++)                                     
                tmp_mat->data.fl[k] = (float)(feature_data[k]);                                    

            // predict probability
            cluster_center.channels[0].values.push_back(forest.predict_prob(tmp_mat));

            //convert pcl::PointCloud to sensor_msgs::PointCloud2
            //sensor_msgs::PointCloud2 cloud_out;
            //pcl::toROSMsg(*cloud_cluster, cloud_out);
            //cloud_out.header.frame_id = "laser_1"; //define the output frame_id
            //extracted_clusters.push_back(cloud_out);
        }
        people_pub.publish(cluster_center);

        //end = ros::Time::now() - begin;
        //std::cout << "get cluster time" << end << endl;

        if (MARKER_PUB)
            marker_publish(&cluster_center);
        //ROS_INFO("The total number of the clusters is: %d, and published %d markers", extracted_clusters.size(), marker_array.markers.size());
        //pub.publish(extracted_clusters[0]);
        //ROS_INFO("Published %d markers", marker_array.markers.size());
    }


    void marker_publish(sensor_msgs::PointCloud* cluster_center)
    {
        //using marker for display
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;
        marker.header.frame_id = "laser_1";
        marker.header.stamp = ros::Time();
        marker.ns = "cluster_center";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        int idx = 0;
        for (vector<geometry_msgs::Point32>::iterator i = cluster_center->points.begin(); i!= cluster_center->points.end(); i++)
        {
            //pass the value to the marker
            marker.id = idx;
            marker.pose.position.x = i->x;
            marker.pose.position.y = i->y;
            marker.pose.position.z = 0.0;
            idx++;
            //collect the markers and publish them as MarkerArray
            marker_array.markers.push_back(marker);
        }

        marker_pub.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "EuclideanClusterExtraction");
    ros::NodeHandle nh;
    string forest_file;
    if(!ros::param::get("~forest", forest_file))
    {
        cout << "please assign forest file" << endl;
        return 0;
    }
    cout << "load forest model: " << forest_file << endl;
    ros::param::get("~publish_marker", MARKER_PUB);
    ros::param::get("~publish_filtered", FILTERED_CLOUD_PUB);

    LaserProcessor lp(nh, forest_file);
    ros::spin();
}

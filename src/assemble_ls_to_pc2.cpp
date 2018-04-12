#include "ros/ros.h"
#include <vector>
#include <boost/range/algorithm.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext.hpp>
#include "tf/transform_listener.h"                                              
#include "sensor_msgs/PointCloud.h"                                             
#include "sensor_msgs/PointCloud2.h"                                            
#include "sensor_msgs/point_cloud_conversion.h"                                 
#include "sensor_msgs/ChannelFloat32.h"                                 
#include "tf/message_filter.h"                                                  
#include "message_filters/subscriber.h"                                         
#include "laser_geometry/laser_geometry.h"

using namespace std;

class LaserScanToPointCloud2{

protected:
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    ros::NodeHandle n_;
    tf::MessageFilter<sensor_msgs::LaserScan>* tf_filter;

    bool new_msg1 = false;
    bool new_msg2 = false;
    
    std::vector<sensor_msgs::LaserScan> msg_1;
    std::vector<sensor_msgs::LaserScan> msg_2;
    sensor_msgs::PointCloud2 pc2_out;



public:
    LaserScanToPointCloud2()
    {
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_1(n_, "/scan_1", 10);
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_2(n_, "/scan_2", 10);
        laser_sub_1.registerCallback(boost::bind(&LaserScanToPointCloud2::scan_1_cb, this, _1));
        tf_filter = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_2, listener_,"laser_1", 10);
        tf_filter->registerCallback(boost::bind(&LaserScanToPointCloud2::scan_2_cb, this, _1));

        ros::Publisher scan_pub = n_.advertise<sensor_msgs::PointCloud2>("cloud_raw", 10);

        while(ros::ok())
        {
            //clock_t t_start, t_end;
            //t_start = clock();
            ros::spinOnce();
            if(new_msg1 || new_msg2)
            {
                new_msg1 = false;
                new_msg2 = false;
                //cout << "got scan1: "<< msg_1.size() << "  got scan2: "<< msg_2.size() << endl;
                
                sensor_msgs::PointCloud cloud_assembled;
                vector<sensor_msgs::ChannelFloat32> channel(2);
                cloud_assembled.channels = channel;
                float index_inc = 0.0; 

                std::vector<sensor_msgs::LaserScan> msg_1_tmp;
                std::vector<sensor_msgs::LaserScan> msg_2_tmp;
                msg_1_tmp = msg_1;
                msg_2_tmp = msg_2;
                msg_1.clear();
                msg_2.clear();

                for (std::vector<sensor_msgs::LaserScan>::const_iterator i=msg_1_tmp.begin(); i!=msg_1_tmp.end(); ++i)
                {
                    // transform to  pointcloud
                    sensor_msgs::PointCloud cloud_1;
                    try
                    {
                        projector_.transformLaserScanToPointCloud("laser_1", *i, cloud_1, listener_);
                    }
                    catch (tf::TransformException& e)
                    {
                        cout << e.what() << endl;
                        return;
                    }

                    // assemble points
                    cloud_assembled.header = cloud_1.header;
                    boost::push_back(cloud_assembled.points, cloud_1.points);

                    // assemble intensity
                    cloud_assembled.channels[0].name = cloud_1.channels[0].name;
                    boost::push_back(cloud_assembled.channels[0].values, cloud_1.channels[0].values);

                    // assemble index
                    cloud_assembled.channels[1].name = cloud_1.channels[1].name;
                    boost::push_back(cloud_assembled.channels[1].values, boost::irange(index_inc, index_inc+cloud_1.channels[1].values.size()));
                    index_inc += cloud_1.channels[1].values.size();
                    // for (unsigned int num=0; num<cloud_1.channels[1].values.size(); num++)
                    //{
                    //    index_inc += 1;
                    //    cloud_assembled.channels[1].values.push_back(index_inc);
                    //}
                }
                // cout << "index_inc: " << index_inc << endl;

                for (std::vector<sensor_msgs::LaserScan>::const_iterator j=msg_2_tmp.begin(); j!=msg_2_tmp.end(); ++j)
                {
                    // transform to  pointcloud
                    sensor_msgs::PointCloud cloud_2;
                    try
                    {
                        projector_.transformLaserScanToPointCloud("laser_1", *j, cloud_2, listener_);
                    }
                    catch (tf::TransformException& e)
                    {
                        std::cout << e.what() << endl;
                        return;
                    }

                    // assemble points
                    boost::push_back(cloud_assembled.points, cloud_2.points);

                    // assemble intensity
                    boost::push_back(cloud_assembled.channels[0].values, cloud_2.channels[0].values);

                    // assemble index
                    boost::push_back(cloud_assembled.channels[1].values, boost::irange(index_inc, index_inc+cloud_2.channels[1].values.size()));
                    index_inc += cloud_2.channels[1].values.size();
                    // for (unsigned int num=0; num<cloud_2.channels[1].values.size(); num++)
                    // {
                    //    index_inc += 1;
                    //    cloud_assembled.channels[1].values.push_back(index_inc);
                    // }
                }
                // cout << "index_inc: " << index_inc << endl;

                if (!sensor_msgs::convertPointCloudToPointCloud2 (cloud_assembled, pc2_out))
                {
                    ROS_ERROR("Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
                }

                scan_pub.publish(pc2_out);
            }
            //calculate time
            // t_end = clock();
            // cout << "Runtime: " << (double)(t_end-t_start)*1000.0/CLOCKS_PER_SEC <<"ms"<< endl;
            ros::Duration(0.3).sleep();


        }
    }

    ~LaserScanToPointCloud2(){};

    void scan_1_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        new_msg1 = true;
        msg_1.push_back(*scan_in);
    }

    void scan_2_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        new_msg2 = true;
        msg_2.push_back(*scan_in);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "assemble_ls_to_pc2");
    LaserScanToPointCloud2 ls2pc;
}

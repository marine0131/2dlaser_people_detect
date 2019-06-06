#include <laser_people_detect/calc_people_features.h>
// if use opencv 2
#include "opencv/cxcore.h"
#include "opencv/cv.h"
// if use opencv 3
// #include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <pcl/common/centroid.h>

// using namespace laser_processor;
using namespace std;
bool DEBUG = false;

vector<float> calcPeopleFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{

    // output features
    vector<float> features;

    // Number of points
    int num_points = cluster->points.size();

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // max distance to centroid and median points
    float max_dist = 0.0;
    float dist_sum = 0.0;
    vector<float> x_median_set;
    vector<float> y_median_set;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
    {
        float d = pow((*i).x - centroid[0], 2) + pow((*i).y - centroid[1], 2);
        dist_sum += d;
        if (d > max_dist)
            max_dist = d;
        x_median_set.push_back((*i).x);
        y_median_set.push_back((*i).y);
    }

    std::sort(x_median_set.begin(), x_median_set.end());
    std::sort(y_median_set.begin(), y_median_set.end());

    float x_median = 0.5 * (*(x_median_set.begin() + (num_points - 1) / 2) + * (x_median_set.begin() + num_points / 2));
    float y_median = 0.5 * (*(y_median_set.begin() + (num_points - 1) / 2) + * (y_median_set.begin() + num_points / 2));
    if (DEBUG)
        cout << "max_dist: " << max_dist << endl;

    features.push_back(max_dist); //the maximum distance from any point to the centroid of the cluster
    //features.push_back(dist_sum); //the sum of the distance from any point to the centroid of the cluster

    //Compute std and avg diff from median
    //double sum_std_diff = 0.0;
    double sum_med_diff = 0.0;

    for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
    {
        // sum_std_diff += pow((*i).x - centroid[0], 2) + pow((*i).y - centroid[1], 2);
        sum_med_diff += sqrt(pow((*i).x - x_median, 2) + pow((*i).y - y_median, 2));
    }

    float std = sqrt(dist_sum / (num_points - 1.0));
    float avg_median_dev = sum_med_diff / num_points;

    features.push_back(std);
    features.push_back(avg_median_dev);

    // Compute Linearity
    CvMat* points = cvCreateMat(num_points, 2, CV_64FC1);
    {
       int j = 0;
       for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
       {
         cvmSet(points, j, 0, (*i).x - centroid[0]);
         cvmSet(points, j, 1, (*i).y - centroid[1]);
         j++;
       }
    }
    cvReleaseMat(&points);

    //CvMat* W = cvCreateMat(2, 2, CV_64FC1);
    //CvMat* U = cvCreateMat(num_points, 2, CV_64FC1);
    //CvMat* V = cvCreateMat(2, 2, CV_64FC1);
    //cvSVD(points, W, U, V);

    //CvMat* rot_points = cvCreateMat(num_points, 2, CV_64FC1);
    //cvMatMul(U, W, rot_points);

    //float linearity = 0.0;
    //for (int i = 0; i < num_points; i++)
    //{
    //  linearity += pow(cvmGet(rot_points, i, 1), 2);
    //}

    //cvReleaseMat(&points);
    //points = 0;
    //cvReleaseMat(&W);
    //W = 0;
    //cvReleaseMat(&U);
    //U = 0;
    //cvReleaseMat(&V);
    //V = 0;
    //cvReleaseMat(&rot_points);
    //rot_points = 0;

    //if (DEBUG)
    //    cout << "linearity: " << linearity << endl;
    //features.push_back(linearity);

    // Compute Circularity
    CvMat* A = cvCreateMat(num_points, 3, CV_64FC1);
    CvMat* B = cvCreateMat(num_points, 1, CV_64FC1);
    {
       int j = 0;
       for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
       {

         cvmSet(A, j, 0, -2.0 * (*i).x);
         cvmSet(A, j, 1, -2.0 * (*i).y);
         cvmSet(A, j, 2, 1);

         cvmSet(B, j, 0, -pow((*i).x, 2) - pow((*i).y, 2));
         j++;
       }
    }
    CvMat* sol = cvCreateMat(3, 1, CV_64FC1);

    cvSolve(A, B, sol, CV_SVD);

    float xc = cvmGet(sol, 0, 0);
    float yc = cvmGet(sol, 1, 0);
    float rc = sqrt(pow(xc, 2) + pow(yc, 2) - cvmGet(sol, 2, 0));

    cvReleaseMat(&A);
    A = 0;
    cvReleaseMat(&B);
    B = 0;
    cvReleaseMat(&sol);
    sol = 0;

    //float circularity = 0.0;
    //for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
    //{
    //  circularity += pow(rc - sqrt(pow(xc - (*i).x, 2) + pow(yc - (*i).y, 2)), 2);
    //}

    //if (DEBUG)
    //    cout << "circularity: " << circularity << endl;
    //features.push_back(circularity);

    // Radius
    if (DEBUG)
        cout << "radius: " << rc << endl;
    features.push_back(rc);

    //min enclosing circle
    vector<cv::Point2f> mat;
    cv::Point2f cent;
    float x_min = 100000, y_min = 100000, x_max = -100000, y_max = -100000;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cluster->begin(); i != cluster->end(); i++)
    {
        // zoom x and y for right detect in minEnclosingCircle, which min output radius is 1
        cent.x = (*i).x * 100;
        cent.y = (*i).y * 100;
        mat.push_back(cent);
        if((*i).x > x_max)
            x_max = (*i).x;
        if((*i).x < x_min)
            x_min = (*i).x;
        if((*i).y > y_max)
            y_max = (*i).y;
        if((*i).y < y_min)
            y_min = (*i).y;
    }
    float enclosing_radius = 0.0;
    cv::minEnclosingCircle(mat, cent, enclosing_radius);
    if (DEBUG)
        cout << "enclosing_radius: " << enclosing_radius << endl;

    // density = num / enclosing_circle area
    float density = 0.0;
    density = num_points / pow(enclosing_radius/100.0, 2);
    features.push_back(density);

    // enclosing rectangle
    float enclosing_rect_ratio = 0.0;
    float enclosing_rect_area = 0.0;
    cv::RotatedRect rRect = cv::minAreaRect(mat);
    enclosing_rect_ratio = rRect.size.width/rRect.size.height;
    if(enclosing_rect_ratio < 1.0)
        enclosing_rect_ratio = 1.0 / enclosing_rect_ratio;

    enclosing_rect_area = rRect.size.width*rRect.size.height;
    features.push_back(enclosing_rect_ratio);
    features.push_back(enclosing_rect_area);

    //manhattan dist
    float manhattan_dist = (x_max-x_min) + (y_max-y_min);
    features.push_back(manhattan_dist);
   

    return features;
}

// int main(int argc, char** argv)
// {
//     vector<float> features;
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::PointXYZ point;
//     cloud.width = 20;
//     cloud.height = 1;
//     cloud.is_dense = false;
//     cloud.points.resize(cloud.width * cloud.height);
//     for (size_t i = 0; i < cloud.points.size(); i++)
//     {
//         float theta = rand()/(RAND_MAX+1.0f);
//         cloud.points[i].x = 5.0 * theta * theta; // 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud.points[i].z = 0.0;
//         cout << cloud.points[i].x << "\t" << cloud.points[i].y << endl;
//     }
//     features = calcLegFeatures(&cloud);
// }

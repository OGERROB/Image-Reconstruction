#ifndef lzy_functions
#define lzy_functions
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <list>
#include <chrono>
#include <cmath>


using namespace std;
using namespace cv;
using namespace Eigen;


void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

void pose_estimation_2d2d (
   const  std::vector<KeyPoint> & keypoints_1,
    const std::vector<KeyPoint> & keypoints_2,
    const std::vector< DMatch > & matches,
    Mat& R, Mat& t );

class imgdata
{ 
public:
  int row;
  int col;
  Eigen::Vector3d pixel;
};

class img_3d_data
{
public:
  Eigen::Vector3d position;
  Eigen::Vector3d pixel;
};



void foretest (const Mat& img_1) ;
void wraptest ( const Mat& img_1 );
vector<imgdata> get_img_data ( const Mat& img_1);

//process of calculation
vector<img_3d_data> get_3d_points ( const Mat& K, 
		     const Mat& img_1, 
		     const Mat& depth );

Point2d pixel2cam ( const Point2d& p, const Mat& K );

Point2d cam2pixel ( const Mat& K, const Vector3d& p_c );

vector<img_3d_data> calculate_new_point_xyz_rgb( const Mat& R, const Mat& t, const vector<img_3d_data>& point_xyz_rgb );


vector<imgdata> reget_2d_points ( const Mat& K, 
		       const vector<img_3d_data>& new_point_xyz_rgb );

//in progress


void draw_new_image(const vector<imgdata>& imgdataset, int flag);


#endif

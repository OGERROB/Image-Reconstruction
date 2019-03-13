#include <functions.h>

using namespace std;
using namespace cv;
using namespace Eigen;



void pose_estimation_2d2d (
   const  std::vector<KeyPoint> & keypoints_1,
    const std::vector<KeyPoint> & keypoints_2,
    const std::vector< DMatch > & matches,
    Mat& R, Mat& t );//放自己写的函数库里面不好使 因为调用了findEssentialMat和recoverPose

int main ( int argc, char** argv )
{
  //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat depth = imread ( argv[2], CV_LOAD_IMAGE_UNCHANGED );
    
    Mat img_2 = imread ( argv[3], CV_LOAD_IMAGE_COLOR );
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 ); 
    Mat R,t;
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;
    //-- 估计两张图像间运动
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );  
    cout << "success" << endl;
    imshow( "original", img_1 );
    imshow( "depth", depth );
    imshow( "real current image", img_2 );
//////////////////////////////////////////////////////////////////////////// prepariation for calculation. done
    vector<imgdata> imgdataset = get_img_data( img_1 );
////////////////////////////////////////////////////////////////////////////process of calculation
    vector<img_3d_data> point_xyz_rgb = get_3d_points( K, img_1, depth );  
    vector<img_3d_data> new_point_xyz_rgb = calculate_new_point_xyz_rgb( R, t, point_xyz_rgb );
    vector<imgdata> new_point_uv_rgb = reget_2d_points ( K, new_point_xyz_rgb ); 
///////////////////////////////////////////////////////////////////////////calculation. done  
////////////////////////////////////////////////////////optimization

    
////////////////////////////////////////////////////////optimization. done
    draw_new_image(new_point_uv_rgb, 1);//0=test  1=real
    waitKey(0);
    return 0;
}

void pose_estimation_2d2d ( const vector< KeyPoint >& keypoints_1, const vector< KeyPoint >& keypoints_2, const vector< DMatch >& matches, Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2d> points1;
    vector<Point2d> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算本质矩阵
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point );

    //-- 从本质矩阵中恢复旋转和平移信息.
    cv::recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
    
}



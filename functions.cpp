#include "functions.h"


vector<img_3d_data> calculate_new_point_xyz_rgb( const Mat& R, const Mat& t, const vector<img_3d_data>& point_xyz_rgb )
{
//   cv:waitKey(0);
  vector<img_3d_data> new_point_xyz_rgb;
  
  for ( img_3d_data pt:point_xyz_rgb )
  {
    img_3d_data new_pixel;

    Mat xyz = ( Mat_<double> ( 3,1 ) << pt.position[0], pt.position[1], pt.position[2] );
    Mat new_xyz = (R.t()*xyz)-t/5000;
//     cout << endl << new_xyz << " is the new 3d position." << endl;
//     waitKey(0);
//     Mat new_xyz = R*xyz+t;
    new_pixel.pixel = pt.pixel;
    new_pixel.position = Vector3d( new_xyz.at<double>(0,0),
						      new_xyz.at<double>(1,0),
						      new_xyz.at<double>(2,0)); 
    new_point_xyz_rgb.push_back(new_pixel);
  }
  
  return new_point_xyz_rgb;
}



vector<imgdata> reget_2d_points ( const Mat& K, const vector<img_3d_data>& new_point_xyz_rgb )
{
  
  vector<imgdata> new_point_uv_rgb;
  for ( img_3d_data pt:new_point_xyz_rgb )
  {
    imgdata new_pt;
    Point2d pt_uv;
    pt_uv = cam2pixel( K, pt.position );
    
    new_pt.row = pt_uv.x;
    new_pt.col = pt_uv.y;
    new_pt.pixel = pt.pixel;
    new_point_uv_rgb.push_back(new_pt);
  }
  return new_point_uv_rgb;
}

vector<img_3d_data> get_3d_points ( const Mat& K, const Mat& img_1, const Mat& depth )
{
   vector<img_3d_data> point_xyz_rgb;
  ushort mindep =10000;
  
  for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
	ushort dtest = depth.at<ushort>(row,col);

	     if ( dtest<100   )   // bad depth
            continue;
	if ( dtest<mindep )
	  mindep = dtest;
      }
    }
    cout << "min" << mindep << endl;
  
  for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
// 	 ushort d1 = depth.ptr<unsigned short> (col)[row];
	 ushort d1 = depth.at<ushort>(row,col);
// 	  if ( d1==0 )   // bad depth
	     if ( d1<100 || d1>5*mindep  )   // bad depth
            continue;
	
	  Point2d pixel_uv (row, col);
	  Point2d pixel_camera = pixel2cam ( pixel_uv, K );
	  double dd1 = double ( d1 ) /5000.0;
	  
	  Vector3d pos ( pixel_camera.x*dd1, pixel_camera.y*dd1, dd1 );
	  img_3d_data pixel3ddata;
	  pixel3ddata.position = pos;
	  pixel3ddata.pixel = Vector3d( 
	    img_1.at<Vec3b>(row, col)[0],
	    img_1.at<Vec3b>(row, col)[1],
	    img_1.at<Vec3b>(row, col)[2]);
	  
	  point_xyz_rgb.push_back(pixel3ddata);
      }
    }
  cout << "3d extraction success" << endl;
  
  //test
  vector<imgdata> testdata;
    
    
 for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
// 	 ushort d1 = depth.ptr<unsigned short> (col)[row];
	 ushort d1 = depth.at<ushort>(row,col);
	  if ( d1<100 || d1>5*mindep )   // bad depth
	    continue;
	 imgdata testpixel;
	 testpixel.row = row;
	 testpixel.col = col;
	 testpixel.pixel = Vector3d( 
	    img_1.at<Vec3b>(row, col)[0],
	    img_1.at<Vec3b>(row, col)[1],
	    img_1.at<Vec3b>(row, col)[2]);
	 
	 testdata.push_back(testpixel);
      }
    }
    
    draw_new_image(testdata,0);
    cv::waitKey(0);
    
    return point_xyz_rgb;
}



void foretest (const Mat& img_1)
{
  //     Mat img_test  = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
   
    Mat img_test(img_1.rows*2, img_1.cols*2, CV_8UC3, Scalar(255, 255, 255));
    
    for ( int times = 0; times<3; times++)
    {
    for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
	img_test.at<Vec3b>(row, col)[(times)%3] = 0;
	img_test.at<Vec3b>(row, col)[(times+1)%3] = 0;
	img_test.at<Vec3b>(row, col)[(times+2)%3] = 255;
      }
    }
    string str="0";
    str+=times;
    imshow( str, img_test );
    }
    
    
    
    
    vector<Eigen::Vector3d> imgdata;

     for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
	Vector3d pixeldata;
	pixeldata[0] = img_1.at<Vec3b>(row, col)[0];
	pixeldata[1] = img_1.at<Vec3b>(row, col)[1];
	pixeldata[2] = img_1.at<Vec3b>(row, col)[2];
	
	imgdata.push_back(pixeldata);
      }
    }
  
       int pos = 0;
       
     Mat img_test2(img_1.cols, img_1.rows, CV_8UC3, Scalar(255, 255, 255));
     
      
     for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
	
	Vector3d pixeldata = * (imgdata.begin() + pos);
	img_test2.at<Vec3b>(col,row)[0] = pixeldata[0];
	img_test2.at<Vec3b>(col,row)[1] = pixeldata[1];
	img_test2.at<Vec3b>(col,row)[2] = pixeldata[2];
	
	pos++;
      }
    }
    
    imshow("result", img_test2);

}

void wraptest ( const Mat& img_1 )
 {
  
     Point2f srcTri[3];
   Point2f dstTri[3];
   
   Mat rot_mat( 2, 3, CV_32FC1 );
   Mat warp_mat( 2, 3, CV_32FC1 );
   Mat warp_dst, warp_rotate_dst;
   
    warp_dst = Mat::zeros( img_1.rows, img_1.cols, img_1.type() );
   
   srcTri[0] = Point2f( 0,0 );
   srcTri[1] = Point2f( img_1.cols - 1, 0 );
   srcTri[2] = Point2f( 0, img_1.rows - 1 );

   dstTri[0] = Point2f( img_1.cols*0.0, img_1.rows*0.33 );
   dstTri[1] = Point2f( img_1.cols*0.85, img_1.rows*0.25 );
   dstTri[2] = Point2f( img_1.cols*0.15, img_1.rows*0.7 );
   
    warp_mat = getAffineTransform( srcTri, dstTri );
    
   warpAffine( img_1, warp_dst, warp_mat, warp_dst.size() );
   imshow( "warp", warp_dst );
  
}


vector<imgdata> get_img_data ( const Mat& img_1 )
{
  vector<imgdata> imgdataset;

  
    for (int row = 0; row<img_1.rows; row++)
    {
      for(int col = 0; col<img_1.cols; col++)
      {
	imgdata pixeldata;
	pixeldata.row = row;
	pixeldata.col = col;
	pixeldata.pixel[0] = img_1.at<Vec3b>(row,col)[0];
	pixeldata.pixel[1] = img_1.at<Vec3b>(row,col)[1];
	pixeldata.pixel[2] = img_1.at<Vec3b>(row,col)[2];
	
	imgdataset.push_back(pixeldata);
	
      }
    }
  
  return imgdataset;
}

void draw_new_image(const vector< imgdata >& imgdataset, int flag)
{
  
  int rowmax = 0;
  int colmax = 0;
  for (auto pixel:imgdataset)
  {
    if(pixel.row>rowmax)
      rowmax = pixel.row;
    if(pixel.col>colmax)
      colmax = pixel.col;
  }
  
  Mat newimage(rowmax+1, colmax+1, CV_8UC3, Scalar(0,0,0));

    for (imgdata pixel:imgdataset)
  {
	newimage.at<Vec3b>(pixel.row,pixel.col)[0] = pixel.pixel[0];
	newimage.at<Vec3b>(pixel.row,pixel.col)[1] = pixel.pixel[1];
	newimage.at<Vec3b>(pixel.row,pixel.col)[2] = pixel.pixel[2];
  }
  
 string str = "new image";
  str+=rand();
  imshow(str, newimage);
  string write = str+".png";
  imwrite(write, newimage);
  
  if (flag==1)
  {
    str+=" after optimization";
    
    for(int times=0;times<10;times++)
    {
  for ( int row=1; row<newimage.rows-1; row++ )
  {
    for ( int col=1; col<newimage.cols-1; col++ )
    {
      if( newimage.at<Vec3b>(row,col)[0]==0
	&&newimage.at<Vec3b>(row,col)[1]==0
	&&newimage.at<Vec3b>(row,col)[2]==0)
      {
	for (int i=-1;i<2;i++)
	{
	  for(int j=-1;j<2;j++)
	  {
	    newimage.at<Vec3b>(row,col)[0] += newimage.at<Vec3b>(row+i,col+j)[0]/24;
	    newimage.at<Vec3b>(row,col)[1] += newimage.at<Vec3b>(row+i,col+j)[1]/24;
	    newimage.at<Vec3b>(row,col)[2] += newimage.at<Vec3b>(row+i,col+j)[2]/24;
	  }
	}	
	for (int i=-2;i<3;i++)
	{
	  for(int j=-2;j<3;j++)
	  {
	    newimage.at<Vec3b>(row,col)[0] += newimage.at<Vec3b>(row+i,col+j)[0]/36;
	    newimage.at<Vec3b>(row,col)[1] += newimage.at<Vec3b>(row+i,col+j)[1]/36;
	    newimage.at<Vec3b>(row,col)[2] += newimage.at<Vec3b>(row+i,col+j)[2]/36;
	  }
	}	
      }    
    }
  }
    }
   
  imshow(str, newimage);
  string write2 = str+".png";
  imwrite(write2, newimage);
  }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

Point2d cam2pixel ( const Mat& K, const Vector3d& p_c )
{
    return Point2d (
               K.at<double> ( 0,0 ) * p_c ( 0,0 ) / p_c ( 2,0 ) + K.at<double> ( 0,2 ),
               K.at<double> ( 1,1 ) * p_c ( 1,0 ) / p_c ( 2,0 ) + K.at<double> ( 1,2 )
           );
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3 
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

ros::Publisher pub_image_;
using namespace cv;
using namespace std;

bool draw_histogram = false;
 
void callback(const sensor_msgs::Image::ConstPtr& msg)
{
  
  sensor_msgs::Image image;
  //sensor_msgs::Image image_out;
  cv_bridge::CvImagePtr cv_ptr; 
  cv_bridge::CvImagePtr cv_ptr_color; 
  
  int morph_elem = 2;
  int morph_size = 5;
  int morph_operator = 2;

  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int thresh = 100;
  int max_thresh = 255;
  RNG rng(12345);
  float max_circle_y = 10000;
  int circle_i = 0;
  
  
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  cv_ptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  
  image = *msg;
  
  int height = image.height;
  int width = image.width;
  
  
  
  medianBlur(cv_ptr->image, cv_ptr->image, 10);
  threshold(cv_ptr->image, cv_ptr->image, 250, 255,3 );
  
  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( cv_ptr->image, cv_ptr->image, morph_operator, element );
  

  Canny( cv_ptr->image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  
  for( int i = 0; i< contours.size(); i++ )
  {
    if(mc[i].y<max_circle_y)
    {
      max_circle_y = mc[i].y;
      circle_i = i;
    }  
  }
      
    
  if( max_circle_y != 10000)
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( cv_ptr_color->image, contours, circle_i, color, 2, 8, hierarchy, 0, Point() );
    circle( cv_ptr_color->image, mc[circle_i], 4, color, -1, 8, 0 );
  }
    
  
  pub_image_.publish(cv_ptr_color->toImageMsg());

  
   
  
  ////////////////// HISTOGRAM //////////////////////////////////////    
  if( draw_histogram &&  max_circle_y != 10000) 
  {
    int histSize = 256;
    Mat hist;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;
    
    calcHist( &cv_ptr_color->image, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    
    
      for( int i = 1; i < histSize; i++ )
    {
	line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
			Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
			Scalar( 255, 0, 0), 2, 8, 0  );
    }
    
    
      
    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );

    waitKey(0);
  }	
  //////////////////////////////////////////////////////////////////////  

  
   
  
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  
  pub_image_ = nh.advertise< sensor_msgs::Image >("/processed_image/image_raw",  1);
  ros::Subscriber sub = nh.subscribe< sensor_msgs::Image >("/mv_25001872/image_raw", 1, callback);  
  
  ros::Rate loop_rate(20);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
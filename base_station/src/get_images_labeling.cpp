#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif
// #include <compressed_image_transport/codec.h>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include "ros/ros.h"
#include <limits>
#include <string>
#include <vector>
#include <fstream>

#include <functions/functions.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

void decodeAndWriteCompressed(const sensor_msgs::CompressedImage& message, ofstream &file, float rotate, float delta_rot, bool positive = false);

void writeImage(const cv::Mat &image, ofstream &file);

int main(int argc, char **argv) 
{
  
  rosbag::Bag bag;
  std::string camera("/front");
  std::string camera_2("/back");
  if (argc < 4) {
    cerr << "Usage: " << argv[0] << " <bag file> <input_file> [<skip first n images> [<camera_name> [<camera_name>]]]  \n";
    return -1;
  }
  if (argc > 4) {
    camera = string(argv[4]);
    cout << "Using camera: " << camera << endl;
  }
  if (argc > 5) {
    camera_2 = string(argv[5]);
    cout << "Using camera 2: " << camera_2 << endl;
  }
  std::string filename(argv[2]);
  
  int ignore = -1;
  if (argc > 3) {
    ignore = atoi(argv[3]);
    cout << "Ignoring ids with less than: " << ignore << endl;
  }
  
  std::vector<std::vector <double> > M;
  functions::getMatrixFromFile(filename, M);
  
//   std::string compressed_topic = camera + "/depth_registered/image_raw/compressedDepth";
  std::string rgb_topic1 = camera + "/rgb/image_raw/compressed";
  std::string rgb_topic2 = camera_2 + "/rgb/image_raw/compressed";
  try {
    ofstream pos_depth_file("positive_depth");
    ofstream neg_depth_file("negative_depth");
    bag.open(string(argv[1]), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(rgb_topic2);
    topics.push_back(rgb_topic1);
    

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool initialized = false;
    bool _positive = false;
    bool ignoring = true;
    foreach(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::CompressedImage::Ptr im = m.instantiate<sensor_msgs::CompressedImage>();
      if (im != NULL) {
        if (m.getTopic() == rgb_topic1) {
          int seq = im->header.seq;
          
          if (seq > ignore) {
            ignoring = false;
            
          } else
            continue;
          
          cout << "ID = " << seq << "\t Stamp = " << im->header.stamp.sec << "." << im->header.stamp.nsec << "\n";
          
          _positive = false;
          
          int32_t sec = im->header.stamp.sec;
          for (unsigned int i=0; i < M.size(); i++) {
            
            if (M[i][0] <= sec and M[i][1] >= sec) {
              _positive = true;
	      
	      Mat decompressed;
	      try
	      {
		// Decode image data
		decompressed = cv::imdecode(im->data, CV_LOAD_IMAGE_UNCHANGED);
	      } catch (exception &e) {
		
	      }
            
	      ostringstream name;
	      name << "image1_" <<im->header.stamp.sec<<"_"<<im->header.stamp.nsec<<".jpeg";
	      imwrite(name.str(), decompressed);
            }
          }
        } 
        if (ignoring)
          continue;
          
        if (_positive && m.getTopic() == rgb_topic2) {
	  Mat decompressed;
	  try
	  {
	    // Decode image data
	    decompressed = cv::imdecode(im->data, CV_LOAD_IMAGE_UNCHANGED);
	  } catch (exception &e) {
	    
	  }
	  
	  ostringstream name;
	  name << "image2_" <<im->header.stamp.sec<<"_"<<im->header.stamp.nsec<<".jpeg";
	  imwrite(name.str(), decompressed);	  
        }
      }
    }

    bag.close();
  } catch (exception &e) {
    cerr << "Exception while manipulating the bag  " << argv[1] << endl;
    cerr << "Content " << e.what() << endl;
    return -2;
  }

  
  return 0;
}
              

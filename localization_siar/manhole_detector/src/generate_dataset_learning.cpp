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

void decodeAndWriteCompressed(const sensor_msgs::CompressedImage& message, ofstream &file, bool rotate = false);

int main(int argc, char **argv) 
{
  
  rosbag::Bag bag;
  std::string camera("/up");
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <bag file> <input_file> [<camera_name>] [<skip first n images>] \n";
    return -1;
  }
  if (argc > 3) {
    camera = string(argv[3]);
    cout << "Using camera: " << camera << endl;
  }
  std::string filename(argv[2]);
  
  int ignore = -1;
  if (argc > 4) {
    ignore = atoi(argv[4]);
    cout << "Ignoring ids with less than: " << ignore << endl;
  }
  
  std::vector<std::vector <double> > M;
  functions::getMatrixFromFile(filename, M);
  
  std::string compressed_topic = camera + "/depth_registered/image_raw/compressedDepth";
  std::string rgb_topic = camera + "/rgb/image_raw/compressed";
  
  
                                                   
  try {
    ofstream pos_depth_file("positive_depth");
    ofstream pos_rgb_file("positive_rgb");
    ofstream neg_depth_file("negative_depth");
    ofstream neg_rgb_file("negative_rgb");
    bag.open(string(argv[1]), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(compressed_topic);
    topics.push_back(rgb_topic);
    

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool initialized = false;
    
    bool _positive = false;
    
    bool ignoring = true;
    
    foreach(rosbag::MessageInstance const m, view)
    {
       
        sensor_msgs::CompressedImage::Ptr im = m.instantiate<sensor_msgs::CompressedImage>();
        if (im != NULL) {
          if (m.getTopic() == rgb_topic) {
            int seq = im->header.seq;
            
            if (seq > ignore) {
              ignoring = false;
              
            } else
              continue;
            
            cout << "ID = " << seq << "\n";
            
            Mat decompressed;
            try
            {
              // Decode image data
              decompressed = cv::imdecode(im->data, CV_LOAD_IMAGE_GRAYSCALE);
            } catch (exception &e) {
              
            }
            
            size_t rows = decompressed.rows;
            size_t cols = decompressed.cols;
            
            Mat downsample(rows/4, cols/4, CV_8UC1);
            cv::resize(decompressed, downsample, Size(), 0.25, 0.25);
            
            
            
            _positive = false;
            for (unsigned int i=0; i < M.size(); i++) {
              if (M[i][0] <= seq and M[i][1] >= seq) {
                _positive = true;
              }
            }
            
           
            if (_positive) {
            
              for (int r = 0; r < downsample.rows; r++)
              {
                for (int c = 0; c < downsample.cols; c++)
                {
                  int pixel = downsample.at<uchar>(r,c);

                  pos_rgb_file << pixel << '\t';
                }
                pos_rgb_file << endl;
                
                
              }
              flip(downsample, downsample, -1);
              for (int r = 0; r < downsample.rows; r++)
              {
                for (int c = 0; c < downsample.cols; c++)
                {
                  int pixel = downsample.at<uchar>(r,c);

                  pos_rgb_file<< pixel << '\t';
                }
                pos_rgb_file << endl;
              }
            } else {
              for (int r = 0; r < downsample.rows; r++)
              {
                for (int c = 0; c < downsample.cols; c++)
                {
                  int pixel = downsample.at<uchar>(r,c);

                  neg_rgb_file<< pixel << '\t';
                }
                neg_rgb_file << endl;
              }
              
            }
          } else {
//             std::cout << "Depth Image.\n";
            if (ignoring)
              continue;
            
            if (_positive) {
              decodeAndWriteCompressed(*im, pos_depth_file, true); 
            } else {
              decodeAndWriteCompressed(*im, neg_depth_file);
            }
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

void decodeAndWriteCompressed(const sensor_msgs::CompressedImage& message, ofstream &file, bool rotate)
{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  std::string image_encoding = message.format.substr(0, message.format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
  {

    // Read compression type from stream
    compressed_depth_image_transport::ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      try
      {
        // Decode image data
        decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
//         return sensor_msgs::Image::Ptr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }
        
        Mat downsample(rows/4, cols/4, CV_32FC1);
        cv::resize(cv_ptr->image, downsample, Size(), 0.25, 0.25);
        
        for (int r = 0; r < downsample.rows; r++)
        {
          for (int c = 0; c < downsample.cols; c++)
          {
            float pixel = downsample.at<float>(r,c);

            file << pixel << '\t';
          }
          file << endl;
        }
        
        if (rotate) {
          flip(downsample, downsample, -1);
          
          for (int r = 0; r < downsample.rows; r++)
          {
            for (int c = 0; c < downsample.cols; c++)
            {
              float pixel = downsample.at<float>(r,c);

              file << pixel << '\t';
            }
            file << endl;
          }
          
        }

        
        
//         return cv_ptr->toImageMsg();
      }
    }
    else
    {
      // Decode raw image
      try
      {
        cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
//         return sensor_msgs::Image::Ptr();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0))
      {
        // Publish message to user callback
//         return cv_ptr->toImageMsg();
      }
    }
  }
}


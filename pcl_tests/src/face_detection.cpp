#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  private:
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_sub_;
  //    image_transport::Publisher image_pub_;
      ros::Publisher roi_pub_;

      cv::CascadeClassifier face_cascade;
  
  public:
    ImageConverter(): it_(nh_),
                      face_cascade("haarcascade_frontalface_alt.xml")
//                      face_cascade("haarcascade_profileface.xml")
    {
      image_sub_ = it_.subscribe("/wide_stereo/right/image_color", 1, &ImageConverter::imageCB, this);
 //     image_pub_ = it_.advertise("/image_converter/output_video", 1);
      roi_pub_ = nh_.advertise<sensor_msgs::RegionOfInterest>("face_roi", 10);

      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      std::vector<cv::Rect> faces;
      findFaces(cv_ptr->image, faces);

      for (size_t i = 0; i < faces.size(); ++i)
      {
        sensor_msgs::RegionOfInterest roi;
        roi.x_offset = faces[i].x;
        roi.y_offset = faces[i].y;
        roi.height = faces[i].height;
        roi.width = faces[i].width;
        roi_pub_.publish(roi);
      }

      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //cv::waitKey(3);
      
//      image_pub_.publish(cv_ptr->toImageMsg());
    }

    void findFaces(cv::Mat frame, std::vector<cv::Rect> &faces)
    {
        cv::Mat frame_gray;
        cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
        cv::equalizeHist( frame_gray, frame_gray );

        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

        for( size_t i = 0; i < faces.size(); i++ )
        {
          cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
          cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
        }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

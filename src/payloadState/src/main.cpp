#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

cv::Mat img;

cv::Mat detectMarker(cv::Mat inputImage)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    return outputImage;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat outImg = detectMarker(img);

    // detect aruco marker here
    cv::imshow("view", img);
    cv::imshow("aruco_marker", outImg);
    cv::waitKey(1);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/iris_depth_camera/c920/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
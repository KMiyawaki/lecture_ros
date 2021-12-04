#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

class ImageProcessingNode
{
public:
  ImageProcessingNode()
  {
    this->sub_img = n.subscribe("/video_source/raw", 10, &ImageProcessingNode::imageCallback, this);
    this->pub_result_img = n.advertise<sensor_msgs::Image>("/image_processing/result_image", 10);
    this->pub_result = n.advertise<std_msgs::String>("/image_processing/result", 10);
  }
  virtual ~ImageProcessingNode() {}

  void imageCallback(const sensor_msgs::Image::ConstPtr &msg_image)
  {
    try
    {
      // ROS の画像を OpenCV で扱える形式に変換する
      cv::Mat cv_image = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8)->image;
      // 何等かの画像処理をする
      cv::Size size = cv_image.size();
      ROS_INFO("%s: Recv image (%d x %d)", __FUNCTION__, size.width, size.height);
      cv::Mat cv_hsv;
      cv::Mat cv_mask;
      cv::Mat cv_result(size, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::cvtColor(cv_image, cv_hsv, cv::COLOR_BGR2HSV);
      // HSV 画像から特定の H の値を持つ画素を抽出する。この場合は赤色。
      cv::inRange(cv_hsv, cv::Scalar(0, 120, 120), cv::Scalar(40, 255, 255), cv_mask); // ここのパラメータを調整する
      cv_result.setTo(cv::Scalar(0, 0, 255), cv_mask);                                 // 抽出された部分を赤く塗りつぶす。
      int red_pixels = cv::countNonZero(cv_mask);                                      // 赤の画素数を数える。
      // OpenCV の画像を ROS の形式に変換して Publish する
      sensor_msgs::ImagePtr msg_result_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, cv_result).toImageMsg();
      this->pub_result_img.publish(msg_result_image);
      // 画素数情報を文字列にして Publish する
      std_msgs::String msg_result;
      std::stringstream sst;
      sst << "R " << red_pixels;
      msg_result.data = sst.str();
      this->pub_result.publish(msg_result);
      ROS_INFO("%s: %s", __FUNCTION__, msg_result.data.c_str());
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub_img;
  ros::Publisher pub_result_img;
  ros::Publisher pub_result;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_processing");
  ROS_INFO("C19XXX"); // 自分の学番に変更しておくこと。
  ImageProcessingNode node;
  ros::spin();
  return 0;
}

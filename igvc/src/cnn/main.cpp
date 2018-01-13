#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <ros/publisher.h>
#include <igvc/CVUtils.hpp>

image_geometry::PinholeCameraModel cam;
ros::Publisher line_cloud_pub;
tf::StampedTransform transform;

void img_callback(const cv::Mat msg_img)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cv::Mat src_img = msg_img;
  cv::Mat colors[3];
  split(src_img, colors);
  src_img = colors[0];
  cloud = toPointCloud(transform, MatToContours(src_img), cam, "usb_cam_center");

  line_cloud_pub.publish(cloud);
}

void info_img_callback(const sensor_msgs::ImageConstPtr& msg,
                       const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr cv_copy;
  cv_copy = cv_bridge::toCvCopy(msg, "");
  cv::Mat img = cv_copy->image;

  img = ResizeCameraImage(img, 640, 360);
  sensor_msgs::CameraInfo cam_info_rsz = ResizeCameraInfo(cam_info, 640, 360);

  cam.fromCameraInfo(cam_info_rsz);
  img_callback(img);
}

void transform_callback(const sensor_msgs::ImageConstPtr& msg) {
  tf::TransformListener tf_listener;
  if (tf_listener.waitForTransform("/odom", "/optical_cam_center", ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform("/odom", "/optical_cam_center", ros::Time(0), transform);
  } else {
    ROS_INFO_STREAM("cannot find transform");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nn_reader");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::CameraSubscriber sub = it.subscribeCamera("semantic_segmentation", 1, info_img_callback);
  // TODO topic name below
  ros::Subscriber image_subscriber = nh.subscribe("/topic_name", 1, transform_callback);
  line_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/linecloud", 1);

  ros::spin();

  return 0;
}

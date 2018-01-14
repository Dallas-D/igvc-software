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
bool first = true;

void img_callback(const cv::Mat msg_img)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cv::Mat src_img = msg_img;
  cv::Mat colors[3];
  split(src_img, colors);
  src_img = colors[2];
  cloud = toPointCloud(transform, MatToContours(src_img), cam, "usb_cam_center");

  for(int i = 0; i < cloud->size(); i++) {
    //ROS_INFO_STREAM("X = " << cloud->at(i).x << " Y = " << cloud->at(i).y);
  }

  line_cloud_pub.publish(cloud);
}

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
  sensor_msgs::CameraInfo cam_info_rsz = ResizeCameraInfo(cam_info, 300, 225);

  cam.fromCameraInfo(cam_info_rsz);
  //ROS_INFO("camera info callback");
}

void img_callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!first) {
    cv_bridge::CvImagePtr cv_copy;
    cv_copy = cv_bridge::toCvCopy(msg, "");
    cv::Mat img = cv_copy->image;

    //img = ResizeCameraImage(img, 300, 225);

    img_callback(img);
  }
  first = false;

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

  //image_transport::CameraSubscriber sub = it.subscribeCamera("semantic_segmentation", 1, info_img_callback);

  ros::Subscriber sub_1 = nh.subscribe("/semantic_segmentation", 1, img_callback);
  ros::Subscriber sub_2 = nh.subscribe("/usb_cam_center/camera_info", 1, camera_info_callback);
  line_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/linecloud", 1);

  ros::spin();

  return 0;
}

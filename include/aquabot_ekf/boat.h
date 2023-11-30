#ifndef BOAT_H
#define BOAT_H

#include <rclcpp/node.hpp>
#include <aquabot_ekf/gps2enu.h>
#include <aquabot_ekf/tf_listener.h>
#include <aquabot_ekf/covariances.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ros_gz_interfaces/msg/param_vec.hpp>

using ros_gz_interfaces::msg::ParamVec;

namespace aquabot_ekf
{

struct Boat
{
  rclcpp::Node* node;
  toENU* enu;
  tf2_ros::TransformBroadcaster br;
  TFListener listener;
  
  // forwarded messages
  PoseWithCovarianceStamped pose;
  Imu imu;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub;

  Boat(rclcpp::Node* node, toENU* enu) : node{node}, enu{enu}, br{node}, listener{node}
  {
    // init pose and imu messages
    pose.header.frame_id = "world";
    imu.header.frame_id = "wamv/wamv/imu_wamv_link/imu_wamv_sensor";

    for(auto i: {0, 7})
      pose.pose.covariance[i] = Covariances::xy;
    pose.pose.covariance[14] = Covariances::z;
    for(auto i: {21, 28, 35})
      pose.pose.covariance[i] = Covariances::rpy;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = Covariances::a;
      imu.angular_velocity_covariance[i] = Covariances::w;
      imu.orientation_covariance[i] = Covariances::rpy;
    }

    pose_pub = node->create_publisher<PoseWithCovarianceStamped>("/wamv/gps_pose", rclcpp::SensorDataQoS());
    imu_pub = node->create_publisher<Imu>("/wamv/imu", rclcpp::SensorDataQoS());

  }

  inline void updatePose(const NavSatFix &gps)
  {
    if(!enu->isInit())
      enu->setReference(gps);

    enu->transform(gps.latitude, gps.longitude, gps.altitude,
                   pose.pose.pose.position);

    const auto offset{listener.tryGetTF("wamv/wamv/base_link", "wamv/wamv/gps_wamv_link/navsat")};
    const auto base{listener.tryGetTF("world", "wamv/wamv/base_link")};

    if(offset.has_value() && base.has_value())
    {
      pose.pose.pose.position.z -= 1.62;  // not quite

      pose.header.stamp = node->get_clock()->now();
      pose_pub->publish(pose);
    }


  }

  inline void updateImu(const Imu &imu)
  {
    this->imu.orientation = imu.orientation;
    this->imu.angular_velocity = imu.angular_velocity;
    //this->imu.linear_acceleration = imu.linear_acceleration;
    this->imu.header.stamp = node->get_clock()->now();
    imu_pub->publish(this->imu);
  }


};
}

#endif // BOAT_H

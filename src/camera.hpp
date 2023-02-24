#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <nav_msgs/msg/odometry.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/twist.h>

using namespace BT;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
float data;
int cameraStart;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
std_msgs::msg::Float64 message;

//void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
//{
//  data = msg->data;
//  message.data = data;
//}
void camera_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  cameraStart = msg->data;
}

class Camera : public BT::AsyncActionNode, rclcpp::Node
{
public:
  Camera(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config), Node("camera_bt_pubsub")
      
  {
    node_ = rclcpp::Node::make_shared("camera_bt");
    sub_ = node_->create_subscription<std_msgs::msg::Int32>("cam_event", 10, &camera_callback);
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
//    subscriber_ = node_->create_subscription<std_msgs::msg::Float64>("distance", 10, &topic_callback);

  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("MaximumDistance")};
  }

  virtual BT::NodeStatus tick() override
  {
    float max_distance;
    std::string maximum_distance;
    if (!getInput<std::string>("MaximumDistance", maximum_distance))
    {
      throw BT::RuntimeError("missing required input [MaximumDistance]");
    }
    else
    {
      max_distance = std::stof(maximum_distance);
    }

    while(true)
    {
      //rclcpp::spin_some(node_);
      //if (data < max_distance){
      //  geometry_msgs::msg::Twist msg;
      //  msg.linear.x = 0.0;
      //  msg.linear.y = 0.0;
      //  msg.angular.z = 0.0; 
      //  cmd_pub_->publish(msg);
      //}
    }
    //if(cameraStart == 2){
    //  return BT::NodeStatus::SUCCESS;
    //}
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

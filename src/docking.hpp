#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>
//#include <Windows.h>

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
int msg_pos;
void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  msg_pos = msg->data;
}
class DockingEvent : public BT::AsyncActionNode, rclcpp::Node
{
public:
  DockingEvent(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), Node("minimal_publisher")
  {
    node_ = rclcpp::Node::make_shared("camera_event");
    //cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
    pub_ = node_->create_publisher<std_msgs::msg::Int32>("pos_ang_dock", 5);
    subscriber_ = node_->create_subscription<std_msgs::msg::Int32>("pos_ang_dock", 5, &topic_callback);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("action")};
  }

  virtual BT::NodeStatus tick() override
  {
    std::string action;
    if (!getInput<std::string>("action", action))
    {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [action]");
    }

    int state = 1;
    
    while (action == "ang_pos")
    {
      rclcpp::spin_some(node_);
      std_msgs::msg::Int32 message;
      if (state == 1)
      {
        system("gnome-terminal -e 'ros2 run trolley_init trolley_init'");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        message.data = 3;
        pub_->publish(message);
        state = 2;
      }
      if (msg_pos == 1)
      {
        message.data = 2;
        pub_->publish(message);
        return BT::NodeStatus::SUCCESS;
      }
    }
    while (action == "trol_pos")
    {
      rclcpp::spin_some(node_);
      std_msgs::msg::Int32 message;
      if (state == 1)
      {
        system("gnome-terminal -e 'ros2 run trolley_init trolley_init'");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        message.data = 4;
        pub_->publish(message);
        state = 2;
      }
      if (msg_pos == 5)
      {
        message.data = 2;
        pub_->publish(message);
        return BT::NodeStatus::SUCCESS;
      }
    }
    while (action == "back_pos")
    {
      rclcpp::spin_some(node_);
      std_msgs::msg::Int32 message;
      if (state == 1)
      {
        system("gnome-terminal -e 'ros2 run trolley_init trolley_init'");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        message.data = 6;
        pub_->publish(message);
        state = 2;
      }
      if (msg_pos == 7)
      {
        message.data = 2;
        pub_->publish(message);
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

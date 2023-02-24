#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public BT::AsyncActionNode
{
  public:
    MinimalPublisher(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
      node_ = rclcpp::Node::make_shared("stepper_event");
      //subscriber_ = node_->create_subcription<std_msgs::msg::Int32>("topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
      publisher_ = node_->create_publisher<std_msgs::msg::Int32>("stepper", 10);
    }
    
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("action")};
    }

    virtual BT::NodeStatus tick() override;

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

BT::NodeStatus MinimalPublisher::tick()
{ 
  std_msgs::msg::Int32 message;
  std::string action;
  if (!getInput<std::string>("action", action))
  {
    throw BT::RuntimeError("missing required input [action]");
  }

  if(action == "up")
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    rclcpp::spin_some(node_);
    message.data = 1;
    publisher_->publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(22000));
    message.data = 0;
    publisher_->publish(message);
    return BT::NodeStatus::SUCCESS;
  }
  if(action == "down")
  { 
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    rclcpp::spin_some(node_);
    message.data = 2;
    publisher_->publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(22000));
    message.data = 0;
    publisher_->publish(message);
    return BT::NodeStatus::SUCCESS;
  }
}

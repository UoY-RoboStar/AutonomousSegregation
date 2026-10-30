#pragma once

#include <QComboBox>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Links the state viewer's robot selection to the simulation window.
class RobotSelection final : public QObject
{
public:
  RobotSelection(QComboBox &selector, const rclcpp::Node::SharedPtr &node,
    const std::string &topic)
  : publisher_(node->create_publisher<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).reliable().transient_local()))
  {
    connect(&selector, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this, &selector](int) {publish(selector.currentText().toStdString());});
    publish(selector.currentText().toStdString());
  }

  void clear() {publish("");}

private:
  void publish(std::string name)
  {
    while (!name.empty() && name.front() == '/') {name.erase(name.begin());}
    while (!name.empty() && name.back() == '/') {name.pop_back();}
    std_msgs::msg::String message;
    message.data = name;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#pragma once
#include <mutex>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robot_selection.hpp"

class SelectionHighlighter final : public rclcpp::Node
{
public:
  explicit SelectionHighlighter(const GuiArguments &arguments)
    : Node("cachecons_gui_selection", rclcpp::NodeOptions().use_global_arguments(false)),
      selection_(arguments.robots)
  {
    subscription_ = create_subscription<std_msgs::msg::String>(arguments.selectionTopic,
      rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::String::ConstSharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        selection_.select(message->data);
      });
    if (!transport_.Subscribe("/world/cachecons/pose/info", &SelectionHighlighter::poses, this))
      throw std::runtime_error("Cannot subscribe to Gazebo robot poses");
  }

  void update()
  {
    std::optional<SelectedPosition> position;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      position = selection_.position();
    }
    // A closing viewer's empty message can be lost in DDS teardown, so no publisher
    // means no selection; the stored name is kept for when the viewer reconnects.
    if (subscription_->get_publisher_count() == 0)
      position.reset();
    // Non-blocking; MarkerManager applies it on its Render event. Sending every
    // tick also covers the renderer becoming ready after a latched selection.
    transport_.Request("/cachecons/selection_marker", selectionMarker(position));
  }

private:
  void poses(const gz::msgs::Pose_V &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    selection_.update(message);
  }

  std::mutex mutex_;
  RobotSelection selection_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  // Destroy transport before the selection state its callbacks refer to.
  gz::transport::Node transport_;
};

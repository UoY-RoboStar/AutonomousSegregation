#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cachecons_interfaces/msg/vector_float2.hpp"

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

#include "obstacle_avoidance_stm.hpp"
#include "obstacle_avoidance_stm.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleAvoidanceNode : public rclcpp::Node
{
private:
  //inputs (from topics)
  bool EnableOA;
  bool DisableOA;
  bool NOAMove;
  vector_float_2 NOAMove_value;
  bool closestDistance;
  float closestDistance_value;
  bool closestAngle;
  float closestAngle_value;
public:
  ObstacleAvoidanceNode()
  : Node("obstacle_avoidance_node")
  {
    enableoa_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "EnableOA", 10, std::bind(&ObstacleAvoidanceNode::enableoa_callback, this, _1));
  
    disableoa_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "DisableOA", 10, std::bind(&ObstacleAvoidanceNode::disableoa_callback, this, _1));

    noamove_subscription_ = this->create_subscription<cachecons_interfaces::msg::VectorFloat2>(
      "NOAMove", 10, std::bind(&ObstacleAvoidanceNode::noamove_callback, this, _1));

    closest_distance_subscription_  = this->create_subscription<std_msgs::msg::Float32>(
      "ClosestDistance", 10, std::bind(&ObstacleAvoidanceNode::closest_distance_callback, this, _1));

    closest_angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "ClosestAngle", 10, std::bind(&ObstacleAvoidanceNode::closesest_angle_callback, this, _1));

    oamove_publisher_ = this->create_publisher<cachecons_interfaces::msg::VectorFloat2>("OAMove", 10);

    output = stm_ref1_output_Enum_Channel { oamove_publisher_ = oamove_publisher_ };

    timer_ = this->create_wall_timer(500ms, std::bind(&ObstacleAvoidanceNode::simulation_step,this));

    EnableOA = false;

    stm = ObstacleAvoidanceStm(output);
  }

private:
  void enableoa_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data?"true":"false");
    if (msg->data)
      this->EnableOA = true;
  }

  void disableoa_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data?"true":"false");
    if (msg->data)
      this->DisableOA = true;
  }

  void noamove_callback(const cachecons_interfaces::msg::VectorFloat2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '(%f,%f)'", msg->values[0], msg->values[1]);

    this->NOAMove = true;
    this->NOAMove_value.values[0] = msg->values[0];
    this->NOAMove_value.values[1] = msg->values[1];
    // auto message = cachecons_interfaces::msg::VectorFloat2();
    // message.values[0] = msg->values[0];
    // message.values[1] = msg->values[1];
    // RCLCPP_INFO(this->get_logger(), "Publishing: '(%f,%f)'", message.values[0], message.values[1]);
    // this->oamove_publisher_->publish(message);
  }

  void closest_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
    this->closestDistance = true;
    this->closestDistance_value = msg->data;
  }

  void closesest_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
    this->closestAngle = true;
    this->closestAngle_value = msg->data;
  }

  void simulation_step() {
    RCLCPP_INFO(this->get_logger(), "Running one step of simulation.");
    this->stm.ReadInputs(EnableOA,DisableOA,NOAMove,NOAMove_value,closestDistance,closestDistance_value,closestAngle,closestAngle_value);
    // run steps of the simulation    
    this->stm.step();
    this->stm.ResetInputs();
  }

  // inputs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableoa_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disableoa_subscription_;
  rclcpp::Subscription<cachecons_interfaces::msg::VectorFloat2>::SharedPtr noamove_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr closest_distance_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr closest_angle_subscription_;

  //outputs
  rclcpp::Publisher<cachecons_interfaces::msg::VectorFloat2>::SharedPtr oamove_publisher_;

  //timers
  rclcpp::TimerBase::SharedPtr timer_;

  //stm
  stm_ref1_output_Enum_Channel output;
  ObstacleAvoidanceStm stm;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  printf("Initialising ObstacleAvoidanceNode.\n");
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}

#pragma GCC diagnostic pop
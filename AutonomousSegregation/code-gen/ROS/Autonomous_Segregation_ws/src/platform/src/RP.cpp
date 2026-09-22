#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
	This class defines a ROS node that interprets inputs and outputs of the d-model in terms of the 
	funcionalities provides by the platform simulation. See comments below for suggestions of how to
	implement this.
*/

/* // Some of these imports may be generated (msg types), but others (irobot or turtlebot4) depend on the platform implementation. 
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "turtlebot4_msgs/msg/user_led.hpp"
#include "turtlebot4_msgs/msg/user_button.hpp"
*/
using namespace std::chrono_literals;

class RP: public rclcpp::Node {
  public:
  	/*
  		We can alis some types here to make implementation simpler.
  		For example:
  			using Undock = irobot_create_msgs::action::Undock; 
  	*/
    
    RP(): Node("RP") {
      // Platform inputs
      /*
      	Define here actions, services or topics that are used to pass requests to the platform.
      	For example, the dock action for the turtlebot4 can be instantiated as follows:
      		dock_client_ptr_ = rclcpp_action::create_client<irobot_create_msgs::action::Dock>(this,"dock"); 
      */

      // Software inputs and outputs
      /*
      	Define publishers and subscribers for the inputs and outputs of the RoboSim d-model.
      	For example, the topic DockStart is an output of the software, so the platform must subscribe
      	to the relevant topic as follows:
      		auto DockStart_callback = [this](std_msgs::msg::Bool::UniquePtr msg) -> void {
		    	RCLCPP_INFO(this->get_logger(), "DockStart call received by platform.");
		        dock();
		    };
		    DockStart = this->create_subscription<std_msgs::msg::Bool>("DockStart", 10, DockStart_callback);
		On the other hand, the topic DockEnd is an input of the software, so the platform must publish
		to the relevant topic as follows:
			DockEnd = this->create_publisher<std_msgs::msg::Bool>("DockEnd",10);
      */
    }

    private:
    	/*
			Define class variables. For example, the publisher to the LED actuator:
				rclcpp::Publisher<turtlebot4_msgs::msg::UserLed>::SharedPtr LED;	
		*/
      
		
		/*
			Define operations for the platform operations. For example, the dock operation yields
			the following definitions.
			
			// The dock action provided by the turtlebot4 simulation
			rclcpp_action::Client<irobot_create_msgs::action::Dock>::SharedPtr dock_client_ptr_;
			// the subscription used to receive request for the robot to dock
      		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr DockStart;
      		// the publisher used to send information about the completion of the dock action.
      		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr DockEnd;
      		
      		These fields are used as shown below:
      		
      		void dock() {
		        using namespace std::placeholders;
		        if (!this->dock_client_ptr_->wait_for_action_server()) {
		          RCLCPP_ERROR(this->get_logger(), "Dock action server not available after waiting");
		          rclcpp::shutdown();
		        }
		
		        auto dock_goal_msg = irobot_create_msgs::action::Dock::Goal();
		        RCLCPP_INFO(this->get_logger(), "Sending dock goal");
		
		        auto send_dock_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
		        send_dock_goal_options.feedback_callback =
		          std::bind(&RP::dock_feedback_callback, this, _1, _2);
		        send_dock_goal_options.result_callback =
		          std::bind(&RP::dock_result_callback, this, _1);
		        this->dock_client_ptr_->async_send_goal(dock_goal_msg, send_dock_goal_options);
		  }
	
	      void dock_feedback_callback(
	        GoalHandleDock::SharedPtr,
	        const std::shared_ptr<const Dock::Feedback> feedback)
	      {
	        std::stringstream ss;
	
	        if (feedback->sees_dock) {
	          ss << "Dock is visible.";
	        } else {
	          ss << "Dock is not visible.";
	        }
	        
	        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
	      }
	
	      void dock_result_callback(const GoalHandleDock::WrappedResult & result)
	      {
	        switch (result.code) {
	          case rclcpp_action::ResultCode::SUCCEEDED:
	            break;
	          case rclcpp_action::ResultCode::ABORTED:
	            RCLCPP_ERROR(this->get_logger(), "Dock goal was aborted");
	            return;
	          case rclcpp_action::ResultCode::CANCELED:
	            RCLCPP_ERROR(this->get_logger(), "Dock goal was canceled");
	            return;
	          default:
	            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
	            return;
	        }
	        if (result.result->is_docked) {
	          RCLCPP_INFO(this->get_logger(), "Robot successfully docked.");
	          auto message = std_msgs::msg::Bool();
	          message.data = true;
	          this->DockEnd->publish(message);
	        }
	      }
      		
		*/
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RP>());
  rclcpp::shutdown();
  return 0;
}

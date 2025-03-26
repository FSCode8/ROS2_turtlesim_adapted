#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RunRectangle : public rclcpp::Node
{
  public:
    RunRectangle(const char* input)
    : Node("run_rectangle")
    {
    	float side_length = std::atof(input);

    	rclcpp::QoS qos(10);
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  
	qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);  
	publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

	this->run(side_length);
    }

  private:
    void run(const float side_length)
    {
	float length = side_length;
	float height = side_length;
	
	std::chrono::milliseconds sleeptime = std::chrono::milliseconds(3000);

      	auto message = geometry_msgs::msg::Twist();
      	message.linear.x = length;
      	message.linear.y = 0;
      	message.linear.z = 0;
      	message.angular.x = 0;
      	message.angular.y = 0;
      	message.angular.z = 0;
     	
       	rclcpp::sleep_for(std::chrono::milliseconds(500));	
	RCLCPP_INFO(this->get_logger(), "Start of function");
     	RCLCPP_INFO(this->get_logger(), "Side length: %.2f", side_length);
      	publisher_->publish(message);
	rclcpp::sleep_for(sleeptime);

	RCLCPP_INFO(this->get_logger(), "1");
      	
	message.linear.x = 0;
      	message.linear.y = height;
      	publisher_->publish(message);
	rclcpp::sleep_for(sleeptime);
      
	RCLCPP_INFO(this->get_logger(), "2");
      	
	message.linear.x = -length;
      	message.linear.y = 0;
      	publisher_->publish(message);
	rclcpp::sleep_for(sleeptime);
	
	RCLCPP_INFO(this->get_logger(), "3");
    
      	message.linear.x = 0;
      	message.linear.y = -height;
      	publisher_->publish(message);
	rclcpp::sleep_for(sleeptime);
     
	RCLCPP_INFO(this->get_logger(), "4");
	RCLCPP_INFO(this->get_logger(), "End of function");
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc <= 1)
  {
    std::cout << "Warning: No side length inserted!\nProgram is started with default value 1.0.\n";
    const char* defaultValue = "1.0"; 
    rclcpp::spin_some(std::make_shared<RunRectangle>(defaultValue));
  } 
  else
  {
    rclcpp::spin_some(std::make_shared<RunRectangle>(argv[1]));
  }
  rclcpp::shutdown();
  return 0;
}

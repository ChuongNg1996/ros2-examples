/*
LINK: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html 
LINK: https://automaticaddison.com/create-a-publisher-and-subscriber-in-c-ros-2-foxy-fitzroy/ 
LINK: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html 
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msg_srv_ex/msg/num.hpp"  // Custom Message

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<msg_srv_ex::msg::Num>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const msg_srv_ex::msg::Num::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);
    }
    rclcpp::Subscription<msg_srv_ex::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
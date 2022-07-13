/*
LINK: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html 
LINK: https://automaticaddison.com/create-a-publisher-and-subscriber-in-c-ros-2-foxy-fitzroy/ 
LINK: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html 
*/


#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions

#include "rclcpp/rclcpp.hpp"  //// ROS Client Library for C++, allows usage of common pieces of the ROS 2 system.
#include "std_msgs/msg/string.hpp" //  built-in message type of ROS.
#include "cpp_pubsub/msg/num.hpp"  // Custom Message

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

/* The next line creates the node class "MinimalPublisher" by inheriting from "rclcpp::Node". 
Every "this" in the code is referring to the node.*/
class MinimalPublisher : public rclcpp::Node
{
  public:
    /*The public constructor names the node "minimal_publisher" and initializes message count "count_" to 0.*/
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    { 
      /*Inside the constructor, the publisher is initialized with the String message type, the 
      topic name topic, and the required queue size to limit messages in the event of a backup.*/
      publisher_ = this->create_publisher<cpp_pubsub::msg::Num>("topic", 10);

      /*Next, timer_ is initialized, which causes the timer_callback function to be executed twice a second.*/
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    /*The timer_callback function is where the message data is set and the messages are actually published. */
    void timer_callback()
    {
      auto message = cpp_pubsub::msg::Num();
      message.num = this->count_++;  

      /*RCLCPP_INFO macro ensures every published message is printed to the console.*/
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num); 
      publisher_->publish(message);
    }

    /*Declaration of the timer, publisher, and counter fields.*/
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<cpp_pubsub::msg::Num>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  /*rclcpp::spin starts processing data from the node, including callbacks from the timer.*/
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
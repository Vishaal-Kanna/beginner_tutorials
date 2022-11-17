// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using event = rclcpp::ParameterEventHandler;
using handle = rclcpp::ParameterCallbackHandle;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Parameter description!";

    // The parameter type is INFERRED from the default value, so in this case it
    // would be set to a string type.
    this->declare_parameter("Parameter_Publisher", "ENPM808X Publisher ",
                            param_desc);

    std::string my_param = this->get_parameter("Parameter_Publisher")
                               .get_parameter_value()
                               .get<std::string>();
    // Debug logger level
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "current value of parameter: " << my_param);

    param_subscriber_ = std::make_shared<event>(this);
    auto param_callback_ptr =
        std::bind(&MinimalPublisher::param_callback, this, _1);
    param_handle_ = param_subscriber_->add_parameter_callback(
        "frequency", param_callback_ptr);
    //  the timer_ is initialized with a period of 1000ms, which causes the
    //  timer_callback function to be executed once a second.
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    std::string my_param = this->get_parameter("Parameter_Publisher")
                               .get_parameter_value()
                               .get<std::string>();
    message.data = my_param.c_str() + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  void param_callback(const rclcpp::Parameter& param) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Got updated parameter" << param.get_name().c_str());
    auto topic_callback_ptr =
        std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(500ms, topic_callback_ptr);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<event> param_subscriber_;
  std::shared_ptr<handle> param_handle_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

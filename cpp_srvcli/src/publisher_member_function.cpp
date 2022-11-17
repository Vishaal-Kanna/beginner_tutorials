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

/**
 * @file Detection_module.hpp
 * @authors Vishaal Kanna Sivakumar
 * @brief Publisher class
 * @version 1.0
 * @date 10/07/2022
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;  // for use of time units: "ms", "s"
using std::placeholders::_1;           // for use with binding Class member
using std::placeholders::_2;           // callback function

// topic types
using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;

// service types
using SERVICE = rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr;
using ADDTWOINTS = example_interfaces::srv::AddTwoInts;
using REQUEST =
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>;
using RESPOSE = std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>;

/**
 * @brief Class for publishing a string
 * @param class_data
 *
 */

class MinimalPublisher : public rclcpp::Node {
 public:

  MinimalPublisher() : Node("minimal_publisher"), m_count_(0) {
    /*
     * Create publisher with buffer size of 10 and frequency = 2 hz
     */
    auto topicName = "topic";
    m_publisher_ = this->create_publisher<STRING>(topicName, 10);
    auto topicCallbackPtr = std::bind(&MinimalPublisher::timer_callback, this);
    m_timer_ = this->create_wall_timer(500ms, topicCallbackPtr);

    /*
     * Creates a service server (with a service name = "add_two_ints_v2")
     */
    auto serviceName = "add_two_ints_v2";
    auto serviceCallbackPtr = std::bind(&MinimalPublisher::add, this, _1, _2);
    m_service_ = create_service<ADDTWOINTS>(serviceName, serviceCallbackPtr);
  }

 private:

  size_t m_count_;
  PUBLISHER m_publisher_;
  TIMER m_timer_;
  SERVICE m_service_;

  void timer_callback() {
    // Create the message to publish
    auto message = STRING();
    message.data = "ENPM808X Publisher " + std::to_string(m_count_++);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());

    // Publish the message
    m_publisher_->publish(message);
  }

  void add(REQUEST request, RESPOSE response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld",
                request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]",
                (long int)response->sum);
    auto message = STRING();
    message.data =
        "ENPM808X Publisher AddTwoInts Request " + std::to_string(m_count_++);
    RCLCPP_FATAL_STREAM(this->get_logger(),
                        "Fatal error: " << message.data.c_str());
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error: " << message.data.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());
    RCLCPP_WARN_STREAM(this->get_logger(), "Warning: " << message.data.c_str());
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Debug: " << message.data.c_str());
  }
};

int main(int argc, char* argv[]) {
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}

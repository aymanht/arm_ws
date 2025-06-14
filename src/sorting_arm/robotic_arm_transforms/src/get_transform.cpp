#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robotic_arm_msgs/msg/yolov8_inference.hpp"
#include "robotic_arm_msgs/msg/world_object_inference.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Create a client to spawn a turtle
    

    // Create turtle2 velocity publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("topic", 1);
    publisher_tf = this->create_publisher<robotic_arm_msgs::msg::WorldObjectInference>("world_object_inference", 10);
    subscription_ = this->create_subscription<robotic_arm_msgs::msg::Yolov8Inference>("/Yolov8_Inference", 10, std::bind(&FrameListener::on_timer, this, _1));


  }

private:
  void on_timer(const robotic_arm_msgs::msg::Yolov8Inference & msg)
  {
    std::vector<std::string> tf_data_class_names;
    std::vector<std::string> tf_data_detected_object_positions;
    robotic_arm_msgs::msg::WorldObjectInference tf_data;
    
    std::vector<std::string> names;
    std::vector<std::string> positions;
    names = msg.class_names;
    positions = msg.detected_obj_positions;

    RCLCPP_INFO(this->get_logger(), "Processing %zu detected objects", names.size());
    
    // In simulation mode, bypass transform lookup and use coordinates directly
    if (names.size() > 0 && positions.size() >= names.size() * 3) {
        // Copy class names directly
        tf_data_class_names = names;
        
        // Copy positions directly (they're already in the correct format from the publisher)
        tf_data_detected_object_positions = positions;
        
        RCLCPP_INFO(this->get_logger(), "Using direct coordinates (simulation mode)");
        for (int i = 0; i < names.size(); i++) {
            if (i * 3 + 2 < positions.size()) {
                RCLCPP_INFO(this->get_logger(), "Object %s: X=%s, Y=%s, Z=%s", 
                           names[i].c_str(), 
                           positions[i*3].c_str(), 
                           positions[i*3+1].c_str(), 
                           positions[i*3+2].c_str());
            }
        }
        
        tf_data.class_names = tf_data_class_names;
        tf_data.detected_obj_positions = tf_data_detected_object_positions;
        
        RCLCPP_INFO(this->get_logger(), "Publishing world object inference with %zu objects", tf_data_class_names.size());
        publisher_tf->publish(tf_data);
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid objects detected or insufficient position data");
    }
  }

  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  int i = 0;
  bool turtle_spawned_;
  rclcpp::Subscription<robotic_arm_msgs::msg::Yolov8Inference>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  rclcpp::Publisher<robotic_arm_msgs::msg::WorldObjectInference>::SharedPtr publisher_tf;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
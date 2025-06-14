#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "robotic_arm_msgs/msg/yolov8_inference.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;
const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:

DynamicFrameBroadcaster()
: Node("dynamic_frame_tf2_broadcaster")
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ = this->create_subscription<robotic_arm_msgs::msg::Yolov8Inference>("/Yolov8_Inference", 10, std::bind(&DynamicFrameBroadcaster::transform_callback, this, _1));


}

private:
void transform_callback(const robotic_arm_msgs::msg::Yolov8Inference & msg) const
{
    std::vector<std::string> message;
    std::vector<std::string> names;
    message = msg.detected_obj_positions;
    names = msg.class_names;

    RCLCPP_INFO(this->get_logger(), "Received yolov8_inference message with %zu objects", names.size());
    
    // In simulation mode, we skip transform broadcasting since we're using direct coordinates
    // This prevents conflicts with the direct coordinate approach
    if (names.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Simulation mode: Skipping transform broadcasting, using direct coordinates");
        for (int i = 0; i < names.size(); i++) {
            if (i * 3 + 2 < message.size()) {
                RCLCPP_INFO(this->get_logger(), "Object %s coordinates: X=%s, Y=%s, Z=%s", 
                           names[i].c_str(), 
                           message[i*3].c_str(), 
                           message[i*3+1].c_str(), 
                           message[i*3+2].c_str());
            }
        }
    }
}

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<robotic_arm_msgs::msg::Yolov8Inference>::SharedPtr subscription_;
  
  

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
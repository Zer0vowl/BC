#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

extern "C"{
#include "LeapC.h"
#include "ExampleConnection.h"
}

#define PI 3.14
#define RAD_TO_DEG(x) (((x) * 180) / PI)

using namespace std::chrono;
using namespace std::chrono_literals;

class GestureInterfaceNode : public rclcpp::Node
{
public:
    GestureInterfaceNode() : Node("gesture_interface_node")
    {
        pose_sub_r= this->create_subscription
            <geometry_msgs::msg::PoseArray>("Hand_Map_R", 10,
            std::bind(&GestureInterfaceNode::pose_callback_r, this, std::placeholders::_1));

        auto rpy_callback = [this] -> void
        {
            calculate_rpy(hand_data, dynamic_rpy);
            dynamic_rpy_pub->publish(dynamic_rpy);
        }; 

        gesture_timer = this->create_wall_timer(20ms, rpy_callback);

        dynamic_rpy_pub = this->create_publisher<geometry_msgs::msg::Vector3>("dynamic_rpy", 10);
    }

private:
    geometry_msgs::msg::Pose hand_data;
    geometry_msgs::msg::Vector3 dynamic_rpy;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dynamic_rpy_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_r;
    rclcpp::TimerBase::SharedPtr gesture_timer;

    void calculate_rpy(geometry_msgs::msg::Pose &hand, geometry_msgs::msg::Vector3 &rpy)
    {
        // Extract quaternion components from the hand orientation
        double w = hand.orientation.w;
        double x = hand.orientation.x;
        double y = hand.orientation.y;
        double z = hand.orientation.z;

        double roll_R;
        double pitch_R;
        double yaw_R;

        double ysqr = y * y;

        // Roll (x-axis rotation)
        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + ysqr);
        roll_R = std::atan2(t0, t1);
        roll_R = RAD_TO_DEG(roll_R)*-1;

        // Pitch (y-axis rotation)
        double t2 = +2.0 * (w * y - z * x);
        t2 = ((t2 > 1.0) ? 1.0 : t2);
        t2 = ((t2 < -1.0) ? -1.0 : t2);
        pitch_R = std::asin(t2);
        pitch_R=RAD_TO_DEG(pitch_R)*-1;

        // Yaw (z-axis rotation)
        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (ysqr + z * z);
        yaw_R = std::atan2(t3, t4);
        yaw_R=RAD_TO_DEG(yaw_R)*-1;

        rpy.x = pitch_R;
        rpy.y = roll_R;
        rpy.z = yaw_R;
    }

    void pose_callback_r(geometry_msgs::msg::PoseArray msg)
    {
        hand_data.orientation.x = msg.poses.at(0).orientation.x;
        hand_data.orientation.y = msg.poses.at(0).orientation.z;
        hand_data.orientation.z = msg.poses.at(0).orientation.y;
        hand_data.orientation.w = msg.poses.at(0).orientation.w;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
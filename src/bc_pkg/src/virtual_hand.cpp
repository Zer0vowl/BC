#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;

class VirtualHandNode : public rclcpp::Node
{
public:
    VirtualHandNode() : Node("virtual_hand_node")
    {
        hand_listener = this->create_subscription
            <geometry_msgs::msg::PoseArray>("Hand_Map_R",10,
            std::bind(&VirtualHandNode::read_hand_map, this, std::placeholders::_1));

        hand_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("Virtual_Hand", 10);
        rpy_timer = this->create_wall_timer(200ms, std::bind(&VirtualHandNode::extract_data, this));
    }

private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hand_publisher;
    visualization_msgs::msg::MarkerArray virtual_hand;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr hand_listener;
    geometry_msgs::msg::PoseArray hand_map;

    rclcpp::TimerBase::SharedPtr rpy_timer;
    int marker_id;
    

    void extract_data()
    {
        int original_marker_id = marker_id;
        virtual_hand.markers.clear();

        for (const auto& pose_data : hand_map.poses)
        {
            visualization_msgs::msg::Marker new_marker;  // Create a new marker for each bone
            geometry_msgs::msg::Point p;


            p.x = pose_data.position.x;
            p.y = pose_data.position.z;
            p.z = pose_data.position.y;

            new_marker.points.push_back(p);

            new_marker.pose.position.x = p.x;
            new_marker.pose.position.y = p.y;
            new_marker.pose.position.z = p.z;
                
                

            new_marker.header.frame_id = "map";
            new_marker.ns = std::to_string(marker_id);
            new_marker.id = marker_id;
            marker_id++;
            new_marker.type = visualization_msgs::msg::Marker::SPHERE;
            new_marker.scale.x = 0.02;
            new_marker.scale.y = 0.02;
            new_marker.scale.z = 0.02;
            new_marker.color.r = 1.0;
            new_marker.color.g = 0.0;
            new_marker.color.b = 0.0;
            new_marker.color.a = 0.7;

            virtual_hand.markers.push_back(new_marker);
        }

        for (const auto& new_marker : virtual_hand.markers)
            {
                RCLCPP_INFO(this->get_logger(), "Marker ID: %d, X: %f, Y: %f, Z: %f",
                    new_marker.id, new_marker.points[0].x, new_marker.points[0].y, new_marker.points[0].z);
            }

        marker_id = original_marker_id;  // Reset marker_id for the next hand
        hand_publisher->publish(virtual_hand);
    }

    void read_hand_map(geometry_msgs::msg::PoseArray msg)
    {
        hand_map = msg;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VirtualHandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
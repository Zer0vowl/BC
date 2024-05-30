#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <thread>

extern "C"{
#include "LeapC.h"
#include "ExampleConnection.h"
}


using namespace std::chrono;
using namespace std::chrono_literals;

class GestureInterfaceNode : public rclcpp::Node
{
public:
    GestureInterfaceNode() : Node("gesture_interface_node")
    {
        // Initialize LeapC connection
        OpenConnection();
        while (!IsConnected)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Wait for the connection to complete
        }

        RCLCPP_INFO(this->get_logger(), "Connected.");
        LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
        if (deviceProps)
        {
            RCLCPP_INFO(this->get_logger(), "Using device %s.", deviceProps->serial);
        }

        hand_map_pub_r = this->create_publisher<geometry_msgs::msg::PoseArray>("Hand_Map_R", 10);
        hand_map_pub_l = this->create_publisher<geometry_msgs::msg::PoseArray>("Hand_Map_L", 10);

        present_r = this->create_publisher<std_msgs::msg::Bool>("present_r",10);
        present_l = this->create_publisher<std_msgs::msg::Bool>("present_l",10);

        present_flag_l.data = false;
        present_flag_r.data = false;

        rpy_timer = this->create_wall_timer(20ms, std::bind(&GestureInterfaceNode::publish_data, this));
    }

private:

    geometry_msgs::msg::PoseArray hand_map_r;
    geometry_msgs::msg::PoseArray hand_map_l;


    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hand_map_pub_r;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hand_map_pub_l;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr present_r;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr present_l;

    std_msgs::msg::Bool present_flag_r;
    std_msgs::msg::Bool present_flag_l;

    rclcpp::TimerBase::SharedPtr rpy_timer;
    
    void publish_data()
    {
        LEAP_TRACKING_EVENT *frame = GetFrame();
        if (!frame)
            return;

        for (uint32_t h = 0; h < frame->nHands; h++)
        {

            if(frame->nHands == 1)
            {
                LEAP_HAND *hand = &frame->pHands[h];

                if(hand -> type == eLeapHandType::eLeapHandType_Right)
                {
                    RCLCPP_INFO(this->get_logger(), "RIGHT HAND ARE PRESENT");
                    present_flag_r.data = true;
                    present_flag_l.data = false;
                    extract_data(hand,hand_map_r);
                    hand_map_pub_r->publish(hand_map_r);
                    hand_map_r.poses.clear();
                    std::cout << std::to_string(h) << "R" << std::endl;
                }

                else if(hand -> type == eLeapHandType::eLeapHandType_Left)
                {
                    RCLCPP_INFO(this->get_logger(), "LEFT HAND ARE PRESENT");
                    present_flag_r.data = true;
                    present_flag_l.data = false;
                    extract_data(hand,hand_map_l);
                    hand_map_pub_l->publish(hand_map_l);
                    hand_map_l.poses.clear();
                    std::cout << std::to_string(h) << "L" << std::endl;
                }
                

            }
            else if(frame->nHands ==2)
            {
                LEAP_HAND *hand_r = &frame->pHands[1];
                LEAP_HAND *hand_l = &frame->pHands[0];

                RCLCPP_INFO(this->get_logger(), "BOTH HANDS ARE PRESENT");
                
                std::thread thread_r([this, hand_r]
                {
                    extract_data(hand_r,hand_map_r);
                    std::cout<<"R"<<std::endl;
                });
                

                std::thread thread_l([this,hand_l]
                {
                    extract_data(hand_l,hand_map_l);
                    std::cout<<"L"<<std::endl;
                });

                present_flag_r.data = true;
                present_flag_l.data = true;

                thread_r.join();
                thread_l.join();
                
                hand_map_pub_l->publish(hand_map_l);
                hand_map_pub_r->publish(hand_map_r);
                
                
                hand_map_r.poses.clear();
                hand_map_l.poses.clear();

                std::cout << std::to_string(h) << "LR" << std::endl;
            }
            else
            {
                std::cout << std::to_string(h) << "NO HAND PRESENT" << std::endl;
                present_flag_r.data = false;
                present_flag_l.data = false;
            }

            std::cout << std::to_string(h) << "H" << std::endl;

        }   
    }


void extract_data(LEAP_HAND *hand, geometry_msgs::msg::PoseArray &hand_map_msg)
{
    geometry_msgs::msg::Pose palm_frame;

    palm_frame.position.x = hand->palm.position.x/1000;
    palm_frame.position.y = hand->palm.position.y/1000;
    palm_frame.position.z = hand->palm.position.z/1000;

    palm_frame.orientation.x = -(hand->palm.orientation.x);
    palm_frame.orientation.y = hand->palm.orientation.y;
    palm_frame.orientation.z = hand->palm.orientation.z;
    palm_frame.orientation.w = hand->palm.orientation.w;

    hand_map_msg.poses.push_back(palm_frame);

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 4; j++)
        {

            geometry_msgs::msg::Pose hand_frame;

            auto midpoint = calculateMidpoint(hand->digits[i].bones[j].prev_joint,
                                              hand->digits[i].bones[j].next_joint);


            hand_frame.position.x = midpoint[0]/1000;
            hand_frame.position.y = midpoint[1]/1000;
            hand_frame.position.z = midpoint[2]/1000;
            

            hand_map_msg.poses.push_back(hand_frame);

        }
    }

  //  for (const auto& frame : hand_map_msg.poses)
  //  {
  //      RCLCPP_INFO(this->get_logger(), "COORD, X: %f, Y: %f, Z: %f",
  //                  frame.position.x, frame.position.y, frame.position.z);
  //  }

}

std::vector<float> calculateMidpoint(const LEAP_VECTOR& start, const LEAP_VECTOR& end)
    {
        std::vector<float> midpoint(3); // Create a vector with 3 elements
        midpoint[0] = (start.x + end.x) / 2.0f; // X coordinate of the midpoint
        midpoint[1] = (start.y + end.y) / 2.0f; // Y coordinate of the midpoint
        midpoint[2] = (start.z + end.z) / 2.0f; // Z coordinate of the midpoint

        return midpoint;
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
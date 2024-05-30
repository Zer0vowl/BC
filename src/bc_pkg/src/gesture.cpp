#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <iostream>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>

using namespace px4_msgs::msg;
using namespace std::chrono;
using namespace std::chrono_literals;

enum hand_type
    {
        RIGHT,
        LEFT
    };

enum finger_state
    {
        CLOSED,
        EXTENDED
    };

enum calibration_state
    {
        COLLECTING_DATA_R,
        COLLECTING_DATA_L,
        CALIBRATING_COLLECTED_DATA,
        READING_GESTURES,
        RESET
    };

class Gesture : public rclcpp::Node
{
    public:
    Gesture() : Node("Gesture")
    {   
        
        
        pose_sub_r = this->create_subscription
            <geometry_msgs::msg::PoseArray>("Hand_Map_R", 10,
            std::bind(&Gesture::gesture_callback_r, this, std::placeholders::_1));
        pose_sub_l = this->create_subscription
            <geometry_msgs::msg::PoseArray>("Hand_Map_L", 10,
            std::bind(&Gesture::gesture_callback_l, this, std::placeholders::_1));
        
        gest_pub_r = this->create_publisher
            <std_msgs::msg::String>("Gesture_R", 10);
        gest_pub_l = this->create_publisher
            <std_msgs::msg::String>("Gesture_L", 10);

        calibrated_pub = this->create_publisher
            <geometry_msgs::msg::Pose>("Calibrated_R", 10);
            
        calibrated_r_pub = this->create_publisher
            <std_msgs::msg::Bool>("calibrated_r",10);
        calibrated_l_pub = this->create_publisher
            <std_msgs::msg::Bool>("calibrated_l",10);
        reset_pub = this->create_publisher
            <std_msgs::msg::Bool>("reset",10);
        calibrateds_pub = this->create_publisher
            <std_msgs::msg::Bool>("calibrated",10);


        msg_counter = 0;
        present_r = false;
        present_l = false;
        calibrated_l_flag = false;
        calibrated_r_flag = false;
        current_state = COLLECTING_DATA_R;

        collected_r_data.data = false;
        collected_l_data.data = false;

        reset.data = false;
        calibrated.data = false;



        auto gesture_callback = [this] -> void
        {
            gesture_r.data = "UNKNOWN";
            gesture_l.data = "UNKNOWN";

            switch(current_state)
            {
                case RESET:
                    std::cout << "RESETING CALIBRATION DATA" << std::endl;

                    present_r = false;
                    present_l = false;
                    calibrated_l_flag = false;
                    calibrated_r_flag = false;
                    collected_r_data.data = false;
                    collected_l_data.data = false;

                    pose_data_r.clear();
                    pose_data_l.clear();

                    pose_r.poses.clear();
                    pose_l.poses.clear();

                    current_state = COLLECTING_DATA_R;

                    count_down(3, "CALIBRATION STARTS AGAIN IN:");

                    reset.data = false;
                    reset_pub->publish(reset);

                    break;

                case COLLECTING_DATA_R:
                    if(pose_r.poses.size() == 0)
                    {
                        std::cout << "PLEASE PLACE RIGHT HAND" << std::endl;;
                        break;
                    }
                    else{
                        if(present_r == false)
                        {
                            start = steady_clock::now();
                            present_r = true;
                            break;
                        }
                        else
                        {
                            if((steady_clock::now() - start) <= seconds(2))
                            {
                                msg_counter++;
                                pose_data_r.push_back(pose_r);
                                break;
                            }
                            else
                            {
                                std::cout<<"R_HAND data collected" << std::endl;
                                current_state = COLLECTING_DATA_L;
                                msg_counter = 0;
                                collected_r_data.set__data(true);
                                break;
                            }
                        }
                    }
                    break;
                case COLLECTING_DATA_L:
                    if(pose_l.poses.size() == 0)
                    {
                        std::cout << "PLEASE PLACE LEFT HAND" << std::endl;;
                        break;
                    }
                    else
                    {
                        if(present_l == false)
                        {
                            start = steady_clock::now();
                            present_l = true;
                            break;
                        }
                        else
                        {
                            if((steady_clock::now() - start) <= seconds(2))
                            {
                                msg_counter++;
                                pose_data_l.push_back(pose_l);
                                break;
                            }
                            else
                            {
                                std::cout<<"L_HAND data collected" << std::endl;
                                current_state = CALIBRATING_COLLECTED_DATA;
                                msg_counter = 0;
                                collected_l_data.set__data(true);
                                break;
                            }
                        } 
                    }
                    break;

                case CALIBRATING_COLLECTED_DATA:
                    if((calibrated_r_flag == true) && (calibrated_l_flag == true))
                    {
                        current_state = READING_GESTURES;
                        calibrated.data = true;
                        break;
                    }
                    if (calibrated_r_flag == false)
                    {
                        if(prepare_calibration_array(pose_data_r,right))
                        {
                            std::cout << "STARTING RIGHT HAND CALIBRATION" << std::endl;
                            create_base_hand(pose_data_r, hand_r,right);
                            std::cout << "RIGHT HAND DATA CALIBRATION : COMPLETED" << std::endl;
                            //pose_data_r.clear();
                            calibrated_r_flag = true;
                            break;
                        }
                        else
                        {
                            std::cout << "HAND DATA CALIBRATION : INCOMPLETE" << std::endl;
                            std::cout << "RECALIBRATING RIGHT HAND..." << std::endl;
                            //pose_data_r.clear();
                            collected_r_data.data=false;
                            current_state = RESET;
                            reset.data = true;
                            break;
                        }
                    }
                    if(calibrated_l_flag == false)
                    {
                        if(prepare_calibration_array(pose_data_l,left))
                        {
                            std::cout << "STARTING LEFT HAND CALIBRATION" << std::endl;
                            create_base_hand(pose_data_l, hand_l,left);
                            std::cout << "LEFT HAND DATA CALIBRATION : COMPLETED" << std::endl;
                            //pose_data_l.clear();
                            calibrated_l_flag = true;
                            break;
                        }
                        else
                        {
                            std::cout << "HAND DATA CALIBRATION : INCOMPLETE" << std::endl;
                            std::cout << "RECALIBRATING LEFT HAND..." << std::endl;
                            //pose_data_l.clear();
                            collected_l_data.data=false;
                            current_state = RESET;
                            reset.data = true;
                            break;
                        }
                    }
                    break;

                case READING_GESTURES:

                    if(pose_r.poses.size() == 0 && pose_l.poses.size() != 0)
                    {
                        std::cout << "READING L HAND GESTURE" << std::endl;
                        std::cout << "L" << std::endl;
                        process_gesture(pose_l,hand_l,gesture_l, left);
                        std::cout << "L" << std::endl;
                        gesture_r.data = "UNKNOWN";
                        pose_l.poses.clear();
                        break;
                    }
                    else if(pose_r.poses.size() != 0 && pose_l.poses.size() == 0)
                    {
                        std::cout << "READING R HAND GESTURE" << std::endl;
                        std::cout << "R" << std::endl;
                        process_gesture(pose_r,hand_r,gesture_r,right);
                        std::cout << "R" << std::endl;
                        gesture_l.data = "UNKNOWN";
                        pose_r.poses.clear();
                        break;
                    }

                    if(pose_r.poses.size() != 0 && pose_l.poses.size() != 0)
                    {
                        std::cout << "READING GESTURE FROM BOTH HANDS" << std::endl;

                        std::thread thread_r([this]
                        {
                            std::cout << "RRRR" << std::endl;
                            process_gesture(pose_r,hand_r,gesture_r,right);
                            std::cout << "RRRR" << std::endl;
                        });
                        
                        std::thread thread_l([this]
                        {
                            std::cout << "LLLL" << std::endl;
                            process_gesture(pose_l,hand_l,gesture_l,left);
                            std::cout << "LLLL" << std::endl;
                        });

                        thread_r.join();
                        thread_l.join();
                        
                        pose_r.poses.clear();
                        pose_l.poses.clear();
                        
                        break;
                    }

                    break;
            }

            reset_pub->publish(reset);
            calibrateds_pub->publish(calibrated);
            gest_pub_r->publish(gesture_r);
            gest_pub_l->publish(gesture_l);
            std::cout<<gesture_r.data<<std::endl;
            std::cout<<gesture_l.data<<std::endl;
            calibrated_l_pub->publish(collected_l_data);
            calibrated_r_pub->publish(collected_r_data);
            publish_calibrated_control_hand();
            
            std::cout<<std::to_string(msg_counter)<<std::endl;
        };

        gesture_timer = this->create_wall_timer(20ms, gesture_callback);       
    }
    private:

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_r;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_l;


    geometry_msgs::msg::PoseArray pose_r;
    geometry_msgs::msg::PoseArray pose_l;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gest_pub_r;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gest_pub_l;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr calibrated_pub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibrated_r_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibrated_l_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibrateds_pub;

    std_msgs::msg::Bool collected_r_data;
    std_msgs::msg::Bool collected_l_data;
    std_msgs::msg::Bool calibrated;

    geometry_msgs::msg::Pose calibrated_palm;
    
    
    float speed;
    float turn_angle;

    float pinky;
    float ring;
    float middle;
    float index;
    float thumb;
    std::vector<float> hand_r;
    std::vector<float> hand_l;
    std::string gest;

   std::chrono::_V2::steady_clock::time_point start;


    int init_flg;

    hand_type right = RIGHT;
    hand_type left = LEFT;


    float x_val;
    float y_val;
    float z_val;
    float w_val;

    std_msgs::msg::String gesture_r;
    std_msgs::msg::String gesture_l;

    bool calibrated_r_flag;
    bool calibrated_l_flag;

    bool present_r;
    bool present_l;
    std_msgs::msg::Bool reset;

    int msg_counter;

    std::vector<finger_state> gesture_fist = {CLOSED, CLOSED, CLOSED, CLOSED, CLOSED};
    std::vector<finger_state> gesture_five = {EXTENDED,EXTENDED,EXTENDED,EXTENDED,EXTENDED};

    std::vector<finger_state> gesture_thumb_l = {CLOSED,CLOSED,CLOSED,CLOSED,EXTENDED};
    std::vector<finger_state> gesture_one_l = {CLOSED,CLOSED,CLOSED,EXTENDED,CLOSED};
    std::vector<finger_state> gesture_two_l = {CLOSED,CLOSED,EXTENDED,EXTENDED,CLOSED};

    std::vector<finger_state> gesture_thumb_r = {EXTENDED,CLOSED,CLOSED,CLOSED,CLOSED};
    std::vector<finger_state> gesture_one_r = {CLOSED,EXTENDED,CLOSED,CLOSED,CLOSED};
    std::vector<finger_state> gesture_two_r = {CLOSED,EXTENDED,EXTENDED,CLOSED,CLOSED};

    std::string current_gesture_R;

    rclcpp::TimerBase::SharedPtr gesture_timer;
    rclcpp::TimerBase::SharedPtr speed_timer;

    calibration_state current_state;
    std::vector<geometry_msgs::msg::PoseArray> pose_data_r;
    std::vector<geometry_msgs::msg::PoseArray> pose_data_l;

    void gesture_callback_r(geometry_msgs::msg::PoseArray msg)
    {
        pose_r = msg;

    }
    void gesture_callback_l(geometry_msgs::msg::PoseArray msg)
    {
        pose_l = msg;
    }

    void process_gesture(geometry_msgs::msg::PoseArray &msg,std::vector<float> &hand, std_msgs::msg::String &gesture, hand_type &hand_type)
    {
        std::vector<float> end;
        end.push_back(msg.poses.at(0).position.x);
        end.push_back(msg.poses.at(0).position.y);
        end.push_back(msg.poses.at(0).position.z);

        std::vector<float> pinky_start;
        std::vector<float> index_start;
        std::vector<float> ring_start;
        std::vector<float> thumb_start;
        std::vector<float> middle_start;

        if(hand_type == RIGHT)
        {

            
            pinky_start.push_back(msg.poses.at(20).position.x);
            pinky_start.push_back(msg.poses.at(20).position.y);
            pinky_start.push_back(msg.poses.at(20).position.z);

            
            index_start.push_back(msg.poses.at(8).position.x);
            index_start.push_back(msg.poses.at(8).position.y);
            index_start.push_back(msg.poses.at(8).position.z);

            
            ring_start.push_back(msg.poses.at(16).position.x);
            ring_start.push_back(msg.poses.at(16).position.y);
            ring_start.push_back(msg.poses.at(16).position.z);

            
            thumb_start.push_back(msg.poses.at(4).position.x);
            thumb_start.push_back(msg.poses.at(4).position.y);
            thumb_start.push_back(msg.poses.at(4).position.z);

            
            middle_start.push_back(msg.poses.at(12).position.x);
            middle_start.push_back(msg.poses.at(12).position.y);
            middle_start.push_back(msg.poses.at(12).position.z);

        }
        else
        {
            
            pinky_start.push_back(msg.poses.at(4).position.x);
            pinky_start.push_back(msg.poses.at(4).position.y);
            pinky_start.push_back(msg.poses.at(4).position.z);

            index_start.push_back(msg.poses.at(16).position.x);
            index_start.push_back(msg.poses.at(16).position.y);
            index_start.push_back(msg.poses.at(16).position.z);

            ring_start.push_back(msg.poses.at(8).position.x);
            ring_start.push_back(msg.poses.at(8).position.y);
            ring_start.push_back(msg.poses.at(8).position.z);

            thumb_start.push_back(msg.poses.at(20).position.x);
            thumb_start.push_back(msg.poses.at(20).position.y);
            thumb_start.push_back(msg.poses.at(20).position.z);

            middle_start.push_back(msg.poses.at(12).position.x);
            middle_start.push_back(msg.poses.at(12).position.y);
            middle_start.push_back(msg.poses.at(12).position.z);

        }

        //std::cout << "READING AFTER CALIBRATION" << std::endl;

        float current_pinky; 
        current_pinky = length_3D(end, pinky_start);
        float current_ring; 
        current_ring = length_3D(end, ring_start);;
        float current_middle;
        current_middle = length_3D(end, middle_start);;
        float current_index;
        current_index = length_3D(end, index_start);;
        float current_thumb;
        current_thumb = length_3D(end, thumb_start);;

        std::vector<float> current_hand;

        if(hand_type == RIGHT)
        {
            current_hand.push_back(current_thumb);
            current_hand.push_back(current_index);
            current_hand.push_back(current_middle);
            current_hand.push_back(current_ring);
            current_hand.push_back(current_pinky);
        }
        else
        {
            current_hand.push_back(current_pinky);
            current_hand.push_back(current_ring);
            current_hand.push_back(current_middle);
            current_hand.push_back(current_index);
            current_hand.push_back(current_thumb);
        }

        //std::cout << "STILL READING" << std::endl;

      //  for(auto digit : current_hand)
      //  {
      //      std::cout << std::to_string(digit) << std::endl;
      //  }

        gesture.data = recognize_gesture(current_hand, hand, hand_type);
        //std::cout << "GESTURE_DATA SET" << std::endl;
    }

    void create_base_hand(const std::vector<geometry_msgs::msg::PoseArray> &msg, std::vector<float> &hand,hand_type &hand_type)
    {
        std::vector<float> end_x;
        std::vector<float> end_y;
        std::vector<float> end_z;
        std::vector<float> end_c;

        std::vector<float> orient_x;
        std::vector<float> orient_y;
        std::vector<float> orient_z;
        std::vector<float> orient_w;

        std::vector<float> pinky_start_x;
        std::vector<float> pinky_start_y;
        std::vector<float> pinky_start_z;
        std::vector<float> pinky_start_c;

        std::vector<float> index_start_x;
        std::vector<float> index_start_y;
        std::vector<float> index_start_z;
        std::vector<float> index_start_c;

        std::vector<float> middle_start_x;
        std::vector<float> middle_start_y;
        std::vector<float> middle_start_z;
        std::vector<float> middle_start_c;

        std::vector<float> ring_start_x;
        std::vector<float> ring_start_y;
        std::vector<float> ring_start_z;
        std::vector<float> ring_start_c;

        std::vector<float> thumb_start_x;
        std::vector<float> thumb_start_y;
        std::vector<float> thumb_start_z;
        std::vector<float> thumb_start_c;

        for(int i = 0; i < msg.size(); i++)
        {
            end_x.push_back(msg.at(i).poses.at(0).position.x);
            end_y.push_back(msg.at(i).poses.at(0).position.y);
            end_z.push_back(msg.at(i).poses.at(0).position.z);

            if(hand_type == RIGHT)
            {   
                orient_x.push_back(msg.at(i).poses.at(0).orientation.x);
                std::cout << std::to_string(orient_x.at(i))<<std::endl;
                orient_y.push_back(msg.at(i).poses.at(0).orientation.y);
                std::cout << std::to_string(orient_y.at(i))<<std::endl;
                orient_z.push_back(msg.at(i).poses.at(0).orientation.z);
                std::cout << std::to_string(orient_z.at(i))<<std::endl;
                orient_w.push_back(msg.at(i).poses.at(0).orientation.w);
                std::cout << std::to_string(orient_w.at(i))<<std::endl;

                pinky_start_x.push_back(msg.at(i).poses.at(20).position.x);
                pinky_start_y.push_back(msg.at(i).poses.at(20).position.y);
                pinky_start_z.push_back(msg.at(i).poses.at(20).position.z);

                
                index_start_x.push_back(msg.at(i).poses.at(8).position.x);
                index_start_y.push_back(msg.at(i).poses.at(8).position.y);
                index_start_z.push_back(msg.at(i).poses.at(8).position.z);

                
                ring_start_x.push_back(msg.at(i).poses.at(16).position.x);
                ring_start_y.push_back(msg.at(i).poses.at(16).position.y);
                ring_start_z.push_back(msg.at(i).poses.at(16).position.z);

                
                thumb_start_x.push_back(msg.at(i).poses.at(4).position.x);
                thumb_start_y.push_back(msg.at(i).poses.at(4).position.y);
                thumb_start_z.push_back(msg.at(i).poses.at(4).position.z);

                
                middle_start_x.push_back(msg.at(i).poses.at(12).position.x);
                middle_start_y.push_back(msg.at(i).poses.at(12).position.y);
                middle_start_z.push_back(msg.at(i).poses.at(12).position.z);
            }
            else
            {
                pinky_start_x.push_back(msg.at(i).poses.at(4).position.x);
                pinky_start_y.push_back(msg.at(i).poses.at(4).position.y);
                pinky_start_z.push_back(msg.at(i).poses.at(4).position.z);

                
                index_start_x.push_back(msg.at(i).poses.at(16).position.x);
                index_start_y.push_back(msg.at(i).poses.at(16).position.y);
                index_start_z.push_back(msg.at(i).poses.at(16).position.z);

                
                ring_start_x.push_back(msg.at(i).poses.at(8).position.x);
                ring_start_y.push_back(msg.at(i).poses.at(8).position.y);
                ring_start_z.push_back(msg.at(i).poses.at(8).position.z);

                
                thumb_start_x.push_back(msg.at(i).poses.at(20).position.x);
                thumb_start_y.push_back(msg.at(i).poses.at(20).position.y);
                thumb_start_z.push_back(msg.at(i).poses.at(20).position.z);

                
                middle_start_x.push_back(msg.at(i).poses.at(12).position.x);
                middle_start_y.push_back(msg.at(i).poses.at(12).position.y);
                middle_start_z.push_back(msg.at(i).poses.at(12).position.z);
            }
            
        }

        end_c.push_back(mean(end_x));
        end_c.push_back(mean(end_y));
        end_c.push_back(mean(end_z));

        ring_start_c.push_back(mean(ring_start_x));
        ring_start_c.push_back(mean(ring_start_y));
        ring_start_c.push_back(mean(ring_start_y));

        thumb_start_c.push_back(mean(thumb_start_x));
        thumb_start_c.push_back(mean(thumb_start_y));
        thumb_start_c.push_back(mean(thumb_start_z));

        index_start_c.push_back(mean(index_start_x));
        index_start_c.push_back(mean(index_start_y));
        index_start_c.push_back(mean(index_start_z));

        middle_start_c.push_back(mean(middle_start_x));
        middle_start_c.push_back(mean(middle_start_y));
        middle_start_c.push_back(mean(middle_start_z));

        pinky_start_c.push_back(mean(pinky_start_x));
        pinky_start_c.push_back(mean(pinky_start_y));
        pinky_start_c.push_back(mean(pinky_start_z));

        pinky = length_3D(end_c, pinky_start_c);
        ring = length_3D(end_c, ring_start_c);
        middle = length_3D(end_c, middle_start_c);
        index = length_3D(end_c, index_start_c);
        thumb = length_3D(end_c, thumb_start_c);

        if(hand_type == RIGHT)
        {
            std::cout<<"*******************DATAAAAAA*******************************"<<std::endl;
            calibrated_palm.orientation.x = mean(orient_x);
            std::cout << std::to_string(calibrated_palm.orientation.x)<<"X"<<std::endl;
            calibrated_palm.orientation.y = mean(orient_y);
            std::cout << std::to_string(calibrated_palm.orientation.y)<<"y"<<std::endl;
            calibrated_palm.orientation.z = mean(orient_z);
            std::cout << std::to_string(calibrated_palm.orientation.z)<<"z"<<std::endl;
            calibrated_palm.orientation.w = mean(orient_w);
            std::cout << std::to_string(calibrated_palm.orientation.w)<<"w"<<std::endl;
            std::cout<<"*******************DATAAAAAA*******************************"<<std::endl;

            hand.push_back(thumb);
            hand.push_back(index);
            hand.push_back(middle);
            hand.push_back(ring/2);
            hand.push_back(pinky);
        }
        else
        {
            hand.push_back(pinky);
            hand.push_back(ring/2);
            hand.push_back(middle);
            hand.push_back(index);
            hand.push_back(thumb);
        }
        
    }

    std::string recognize_gesture(std::vector<float> current_hand,std::vector<float> &hand, hand_type &hand_type)
    {
        std::vector<finger_state> current_hand_state;

        for(int i = 0; i < 5; i++)
        {   

            if((current_hand.at(i) >= hand.at(i)/1.5))
                {
                    current_hand_state.push_back(EXTENDED);
                    std::cout << std::to_string(current_hand.at(i)) << "CURRENT" << std::endl;
                    std::cout << std::to_string(hand.at(i)) << "ORIGINAL" << std::endl;
                }
            else
                {
                    current_hand_state.push_back(CLOSED);
                    std::cout << std::to_string(current_hand.at(i)) << "CURRENT" << std::endl;
                    std::cout << std::to_string(hand.at(i)) << "ORIGINAL" << std::endl;
                }

            std::cout << std::to_string(current_hand_state.at(i)) << std::endl;
        }

        if(current_hand_state == gesture_fist)
        {
            gest = "FIST";
        }
        else if(current_hand_state == gesture_five)
        {
            gest = "FIVE";
        }
        else if(current_hand_state == gesture_thumb_r && hand_type == RIGHT)
        {
            gest = "THUMB";
        }
        else if(current_hand_state == gesture_thumb_r && hand_type == LEFT)
        {
            gest = "THUMB";
        }
        else if(current_hand_state == gesture_one_r && hand_type == RIGHT)
        {
            gest = "ONE";
        }        
        else if(current_hand_state == gesture_one_r && hand_type == LEFT)
        {
            gest = "ONE";
        }
        else if(current_hand_state == gesture_two_r && hand_type == RIGHT)
        {
            gest = "TWO";
        }
        else if(current_hand_state == gesture_two_r && hand_type == LEFT)
        {
            gest = "TWO";
        }
        else
        {
            gest = "UNKNOWN";
        }

        current_hand_state.clear();
        current_hand.clear();

        return gest;
    }

    float length_3D(std::vector<float> start, std::vector<float> end)
    {
        float distance = std::sqrt(std::pow((end[0] - start[0]),2) + std::pow((end[1] - start[1]),2) + std::pow((end[2] - start[2]),2));
        return distance;
    }

    float mean(std::vector<float> vect)
    {
        float mean = 0.0;

        for(float val : vect)
        {
            mean += val;
        }

        mean = mean/vect.size();
        return mean;
    }

    double calculatePositionDifference(geometry_msgs::msg::Pose& pose1, geometry_msgs::msg::Pose& pose2) 
    {
        double dx = std::abs(pose1.position.x) - std::abs(pose2.position.x);
        double dy = std::abs(pose1.position.y) - std::abs(pose2.position.y);
        double dz = std::abs(pose1.position.z) - std::abs(pose2.position.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool calculate_difference(std::vector<double> diff_vector)
    {
        bool diff_flag = false;
        for(int i = 20; i < 69 && !diff_flag; i++)
        {
            for(int j = i+1; j < 70; j++)
            {
                if(diff_vector.at(i) - diff_vector.at(j) > 0.05)
                {
                    diff_flag = true;
                    std::cout<<"fail"<<std::endl;
                    break;
                }
            }
        }
        return diff_flag;
    }

    bool prepare_calibration_array(std::vector<geometry_msgs::msg::PoseArray> &pose_data, hand_type &hand_type)
    {
        //msg_counter++;
        bool calibration_flag = true;
        //auto now = steady_clock::now();

        std::vector<double> diff_vector_palm;
        std::vector<double> diff_vector_pinky;
        std::vector<double> diff_vector_ring;
        std::vector<double> diff_vector_middle;
        std::vector<double> diff_vector_index;
        std::vector<double> diff_vector_thumb;

      /*  while(duration_cast<seconds>(steady_clock::now() - start) <= seconds(2))
        {
            if(msg.poses.size()==0)
            {
                std::cout <<"calibration failed" << std::endl;
                calibration_flag = false;
            }
      //      std::cout << "Pushing back data.." << std::endl;
            pose_data.push_back(msg);

            auto now = steady_clock::now();

            if(duration_cast<seconds>(now - last_second_mark) >= seconds(1)) {
                last_second_mark = now; // Update the last second mark to the current time
                auto seconds_passed = duration_cast<seconds>(now - start).count();
                std::cout << seconds_passed << std::endl;
            }
        }*/

     //   std::cout << "Data collection completed" << std::endl;

        for(int i = 0; i < pose_data.size()-1; i++)
        {
            if(hand_type == RIGHT)
            {
                diff_vector_palm.push_back(calculatePositionDifference(pose_data.at(i).poses.at(0),pose_data.at(i+1).poses.at(0)));
                diff_vector_pinky.push_back(calculatePositionDifference(pose_data.at(i).poses.at(20),pose_data.at(i+1).poses.at(20)));
                diff_vector_ring.push_back(calculatePositionDifference(pose_data.at(i).poses.at(16),pose_data.at(i+1).poses.at(16)));
                diff_vector_middle.push_back(calculatePositionDifference(pose_data.at(i).poses.at(12),pose_data.at(i+1).poses.at(12)));
                diff_vector_index.push_back(calculatePositionDifference(pose_data.at(i).poses.at(8),pose_data.at(i+1).poses.at(8)));
                diff_vector_thumb.push_back(calculatePositionDifference(pose_data.at(i).poses.at(4),pose_data.at(i+1).poses.at(4)));
            }
            else
            {
                diff_vector_palm.push_back(calculatePositionDifference(pose_data.at(i).poses.at(0),pose_data.at(i+1).poses.at(0)));
                diff_vector_pinky.push_back(calculatePositionDifference(pose_data.at(i).poses.at(4),pose_data.at(i+1).poses.at(4)));
                diff_vector_ring.push_back(calculatePositionDifference(pose_data.at(i).poses.at(8),pose_data.at(i+1).poses.at(8)));
                diff_vector_middle.push_back(calculatePositionDifference(pose_data.at(i).poses.at(12),pose_data.at(i+1).poses.at(12)));
                diff_vector_index.push_back(calculatePositionDifference(pose_data.at(i).poses.at(16),pose_data.at(i+1).poses.at(16)));
                diff_vector_thumb.push_back(calculatePositionDifference(pose_data.at(i).poses.at(20),pose_data.at(i+1).poses.at(20)));
            }
            
            
  //          std::cout << "Calibrating collected data...." << std::endl;
        }

       // std::cout << "Calibration completed" << std::endl;

        if(calculate_difference(diff_vector_palm))
        {
            calibration_flag = false;
        }
        if(calculate_difference(diff_vector_pinky))
        {
            calibration_flag = false;
        }
        if(calculate_difference(diff_vector_ring))
        {
            calibration_flag = false;
        }
        if(calculate_difference(diff_vector_middle))
        {
            calibration_flag = false;
        }
        if(calculate_difference(diff_vector_index))
        {
            calibration_flag = false;
        }
        if(calculate_difference(diff_vector_thumb))
        {
            calibration_flag = false;
        }

        return calibration_flag;
    }

    void publish_calibrated_control_hand()
    {
        if(calibrated_r_flag == true)
        {
            calibrated_pub -> publish(calibrated_palm);
            std::cout << "PUBLUSHED PALM" << std::to_string(calibrated_palm.orientation.x) << std::to_string(calibrated_palm.orientation.y) 
                                          << std::to_string(calibrated_palm.orientation.z) << std::to_string(calibrated_palm.orientation.w) <<std::endl;
        }
        else
        {
            std::cout << "NOT PUBLISHED" << std::endl;
        }
    }

    void count_down(int lim, std::string message)
    {
        auto start = steady_clock::now();
        auto last_second_mark = start;

        std::cout<< message <<std::endl;
                    
        while(duration_cast<seconds>(steady_clock::now() - start) <= seconds(lim))
        {
            auto now = steady_clock::now();
            if(duration_cast<seconds>(now - last_second_mark) >= seconds(1)) 
            {
                last_second_mark = now; // Update the last second mark to the current time
                auto seconds_passed = duration_cast<seconds>(now - start).count();
                std::cout << seconds_passed << std::endl;
            }
        }
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto gesture = std::make_shared<Gesture>();

    rclcpp::spin(gesture);

    rclcpp::shutdown();
    return 0;
}
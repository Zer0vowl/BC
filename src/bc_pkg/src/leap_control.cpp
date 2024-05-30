#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <iostream>
#include <map>
#include <chrono>
#include <fstream>

using namespace px4_msgs::msg;
using namespace std::chrono;
using namespace std::chrono_literals;

enum Direction
    {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        UP,
        DOWN,
        TURN_R,
        TURN_L,
        NEUTRAL
    };

enum control_state
    {
        RESET,
        CALIBRATING,
        EVALUATING_DATA,
        CONTROL,
        COMPLETED
    };

class LeapControl : public rclcpp::Node
{
    public:
    LeapControl() : Node("leap_control"), data_file("flight_data.txt")
    {   
        //QoS for PX4 connectiom
         static const rmw_qos_profile_t qos_profile = 
        {   
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

        palm_sub= this->create_subscription
            <geometry_msgs::msg::Pose>("Calibrated_R", 10,
            std::bind(&LeapControl::palm_callback, this, std::placeholders::_1));

        pose_sub_r= this->create_subscription
            <geometry_msgs::msg::PoseArray>("Hand_Map_R", 10,
            std::bind(&LeapControl::pose_callback_r, this, std::placeholders::_1));

        rpy_sub = this->create_subscription
            <geometry_msgs::msg::Vector3>("dynamic_rpy", 10,
            std::bind(&LeapControl::get_rpy, this, std::placeholders::_1));

        gest_sub_r = this->create_subscription
            <std_msgs::msg::String>("Gesture_R", 10,
            std::bind(&LeapControl::get_gest_r, this, std::placeholders::_1));
        gest_sub_l = this->create_subscription
            <std_msgs::msg::String>("Gesture_L", 10,
            std::bind(&LeapControl::get_gest_l, this, std::placeholders::_1));
            
        calibrated_r_sub = this->create_subscription
            <std_msgs::msg::Bool>("calibrated_r", 10,
            std::bind(&LeapControl::calibrated_r_callback, this, std::placeholders::_1));
        calibrated_l_sub = this->create_subscription
            <std_msgs::msg::Bool>("calibrated_l", 10,
            std::bind(&LeapControl::calibrated_l_callback, this, std::placeholders::_1));
        reset_sub = this->create_subscription
            <std_msgs::msg::Bool>("reset", 10,
            std::bind(&LeapControl::reset_callback, this, std::placeholders::_1));
        calibrated_sub = this->create_subscription
            <std_msgs::msg::Bool>("calibrated", 10,
            std::bind(&LeapControl::calibrated_callback, this, std::placeholders::_1));

        vehicle_local_position_sub = this->create_subscription
            <VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
            std::bind(&LeapControl::vehicle_position_callback, this, std::placeholders::_1));

        velocity_pub = this->create_publisher
            <geometry_msgs::msg::Twist>("/offboard_velocity_cmd",qos);

        arm_pub = this->create_publisher
            <std_msgs::msg::Bool>("/arm_message",qos);

        current_state = CALIBRATING;
        eval_flag = 0;
        read_gest_counter = 0;
        init_flag = 1;
        mission_flag = 1;
        msg_ctr = 0;
        start_timer = 1;
        eval_stage = 1;
        final_time = 0;


        speed = 1.0;
        turn_angle = 0.5;
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 0.0;

        x_val = 0.0;
        y_val = 0.0;
        z_val = 0.0;
        w_val = 0.0;

       current_mode = "MODE_1";

       lin_coord_map = 
        {
            {"UP", {0,0,1,0}},
            {"DOWN", {0,0,-1,0}},
            {"FORWARD", {0,1,0,0}},
            {"BACKWARD", {0,-1,0,0}},
            {"RIGHT", {-1,0,0,0}},
            {"LEFT", {1,0,0,0}},
            {"TURN_R", {0,0,0,1}},
            {"TURN_L", {0,0,0,-1}},
            {"NEUTRAL", {0,0,0,0}}
        };

        auto gesture_callback = [this] -> void
        {
            switch(current_state)
            {
                case RESET:
                    std::cout<<"RESETING..."<<std::endl;

                    if(reset == false)
                    {
                        current_state = CALIBRATING;
                    }
                    break;
                    
                case CALIBRATING:
                    if(reset == true)
                    {
                        current_state = RESET;
                        break;
                    }

                    if(calibrated == true)
                    {
                        std::cout<<"BOTH HANDS CALIBRATED"<<std::endl;
                        if(eval_flag == 1)
                        {
                            current_state = EVALUATING_DATA;
                            break;
                        }
                        
                        else
                        {
                            current_state = CONTROL;
                            break;
                        }
                    }
                    
                    if(calibrated_r == false)
                    {
                        std::cout<<"CALIBRATING RIGHT HAND"<<std::endl;
                        std::cout<<"PLEASE PLACE SAID HAND OVER THE SENSOR"<<std::endl;

                        if(calibrated_r == true)
                        {
                            std::cout<<"RIGHT HAND SUCCESFULLY CALIBRATED"<<std::endl;
                        }
                        break;
                    }

                    if(calibrated_l == false)
                    {
                        std::cout<<"CALIBRATING LEFT HAND"<<std::endl;
                        std::cout<<"PLEASE PLACE SAID HAND OVER THE SENSOR"<<std::endl;
                        if(calibrated_l == true)
                        {
                            std::cout<<"LEFT HAND SUCCESFULLY CALIBRATED"<<std::endl;
                        }
                        break;
                    }

                break;

            case EVALUATING_DATA:
            {
                if (!data_file.is_open()) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open flight_data.txt for writing.");
                    break;
                }

                if(eval_stage == 1)
                {
                    if(init_flag == 1)
                    {
                        std::cout<<"MAKE THE FOLLOWING GESTURES" << std::endl;
                        std::cout<<"R: FIVE" << std::endl;
                        std::cout<<"L: FIVE" << std::endl;
                        count_down();
                        start = steady_clock::now();
                        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
                        init_flag = 0;
                    }
                    else
                    {
                        if((steady_clock::now() - start) <= seconds(2))
                        {
                            std::cout<<msg_ctr<<std::endl;
                            std::cout<<msg_ctr<<std::endl;
                            write_data_to_file("R","FIVE", gest_r);
                            write_data_to_file("L","FIVE", gest_l);
                            msg_ctr++;
                            std::cout<<msg_ctr<<std::endl;
                            
                        }
                        else
                        {
                            std::cout<<"DATA COLLECTED" << std::endl;
                            eval_stage++;
                            init_flag = 1;
                            msg_ctr=0;
                        }
                    }
                }

                if(eval_stage == 2)
                {
                    if(init_flag == 1)
                    {
                        std::cout<<"MAKE THE FOLLOWING GESTURES" << std::endl;
                        std::cout<<"R: ONE" << std::endl;
                        std::cout<<"L: TWO" << std::endl;
                        count_down();
                        start = steady_clock::now();
                        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
                        init_flag = 0;
                    }
                    else
                    {
                        if((steady_clock::now() - start) <= seconds(2))
                        {
                            std::cout<<msg_ctr<<std::endl;
                            std::cout<<msg_ctr<<std::endl;
                            write_data_to_file("R","ONE", gest_r);
                            write_data_to_file("L","TWO", gest_l);
                            msg_ctr++;
                            std::cout<<msg_ctr<<std::endl;
                            
                        }
                        else
                        {
                            std::cout<<"DATA COLLECTED" << std::endl;
                            eval_stage++;
                            init_flag = 1;
                            msg_ctr=0;
                        }
                    }
                }

                if(eval_stage == 3)
                {
                    if(init_flag == 1)
                    {
                        std::cout<<"MAKE THE FOLLOWING GESTURES" << std::endl;
                        std::cout<<"R: FIST" << std::endl;
                        std::cout<<"L: THUMB" << std::endl;
                        count_down();
                        start = steady_clock::now();
                        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
                        init_flag = 0;
                    }
                    else
                    {
                        if((steady_clock::now() - start) <= seconds(2))
                        {
                            std::cout<<msg_ctr<<std::endl;
                            std::cout<<msg_ctr<<std::endl;
                            write_data_to_file("R","FIST", gest_r);
                            write_data_to_file("L","THUMB", gest_l);
                            msg_ctr++;
                            std::cout<<msg_ctr<<std::endl;
                            
                        }
                        else
                        {
                            std::cout<<"DATA COLLECTED" << std::endl;
                            eval_stage++;
                            init_flag = 1;
                            msg_ctr=0;
                        }
                    }
                }

                if(eval_stage == 4)
                {
                    if(init_flag == 1)
                    {
                        std::cout<<"MAKE THE FOLLOWING GESTURES" << std::endl;
                        std::cout<<"R: TWO" << std::endl;
                        std::cout<<"L: FIST" << std::endl;
                        count_down();
                        start = steady_clock::now();
                        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
                        init_flag = 0;
                    }
                    else
                    {
                        if((steady_clock::now() - start) <= seconds(2))
                        {
                            std::cout<<msg_ctr<<std::endl;
                            std::cout<<msg_ctr<<std::endl;
                            write_data_to_file("R","TWO", gest_r);
                            write_data_to_file("L","FIST", gest_l);
                            msg_ctr++;
                            std::cout<<msg_ctr<<std::endl;
                            
                        }
                        else
                        {
                            std::cout<<"DATA COLLECTED" << std::endl;
                            eval_stage++;
                            init_flag = 1;
                            msg_ctr=0;
                        }
                    }
                }
                if(eval_stage == 5)
                {
                    if(init_flag == 1)
                    {
                        std::cout<<"MAKE THE FOLLOWING GESTURES" << std::endl;
                        std::cout<<"R: THUMB" << std::endl;
                        std::cout<<"L: ONE" << std::endl;
                        count_down();
                        start = steady_clock::now();
                        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
                        init_flag = 0;
                    }
                    else
                    {
                        if((steady_clock::now() - start) <= seconds(2))
                        {
                            std::cout<<msg_ctr<<std::endl;
                            std::cout<<msg_ctr<<std::endl;
                            write_data_to_file("R","THUMB", gest_r);
                            write_data_to_file("L","ONE", gest_l);
                            msg_ctr++;
                            std::cout<<msg_ctr<<std::endl;
                            
                        }
                        else
                        {
                            std::cout<<"DATA COLLECTED" << std::endl;
                            std::cout<<"PROCEEDING TO CONTROL STAGE" << std::endl;
                            eval_stage = 0;
                            init_flag = 0;
                            msg_ctr=0;
                            current_state = CONTROL;
                        }
                    }
                }
                break;
            }
            case CONTROL:
                {
                    arm_state.data = true;
                    arm_message = arm_state;
                    arm_pub->publish(arm_message);


                    if(mission_flag == 1)
                    {

                        if(start_timer == 1)
                        {
                            auto start = steady_clock::now();
                            auto last_second_mark = start;
                            start_timer = 0;
                        }
                        auto now = steady_clock::now();
                        auto seconds_passed = duration_cast<seconds>(now - start).count();
                        std::cout <<"duration:"<< seconds_passed <<"s"<< std::endl;
                        std::cout <<"X:"<< pos_msg.x << std::endl;
                        std::cout <<"Y:"<< pos_msg.y << std::endl;
                        std::cout <<"Z:"<< pos_msg.z << std::endl;
                        if((pos_msg.x <= -7 && pos_msg.x >= -9) && (pos_msg.y <= 5 && pos_msg.y >= 3))
                        {
                            final_time = seconds_passed;
                            current_state = COMPLETED;
                            break;
                        }
                    }


                    mode_switch(gest_l);

                    if(current_mode == "MODE_1")
                    {
                        carth_control();
                    }
                    else
                    {
                        rpy_control();
                    }

                    if(lin_coord_map.find(drone_command) != lin_coord_map.end())
                    {
                        x = std::get<0>(lin_coord_map[drone_command]);
                        y = std::get<1>(lin_coord_map[drone_command]);
                        z = std::get<2>(lin_coord_map[drone_command]);
                        w = std::get<3>(lin_coord_map[drone_command]);
                    }
                    else
                    {
                        x = 0.0;
                        y = 0.0;
                        z = 0.0;
                        w = 0.0;
                        drone_command = "UNKNOWN";
                    }

                    x_val = (x * speed);
                    y_val = (y * speed);
                    z_val = (z * speed);
                    w_val = (w * turn_angle);

                    twist.linear.x = x_val;
                    twist.linear.y = y_val;
                    twist.linear.z = z_val;

                    twist.angular.x = 0.0;
                    twist.angular.y = 0.0;
                    twist.angular.z = w_val;

                    std::cout << gest_r << "R" << std::endl;
                    std::cout << gest_l << "L" << std::endl;
                    std::cout << drone_command << "COMMAND" << std::endl;
                    std::cout << current_mode << "MODE" << std::endl;
                    std::cout << rpy.x << "R" << std::endl;
                    std::cout << rpy.y << "P" << std::endl;
                    std::cout << rpy.z << "Y" << std::endl;
                    std::cout<<"SPEED"<<std::to_string(speed)<<std::endl;
                    break;
                }
            case COMPLETED:
                drone_command = "NEUTRAL";
                velocity_pub->publish(twist);
                arm_message.data = false;
                arm_pub->publish(arm_message);
                std::cout<<"COMPLETED"<<std::endl;
                std::cout<<"FINISHED OBSTACLE COURSE UNDER:"<<final_time<<"s"<<std::endl;

        }
        if(current_state == COMPLETED)
        {
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = -0.5;

            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
        }
        velocity_pub->publish(twist);

    }; 
        gesture_timer = this->create_wall_timer(20ms, gesture_callback);
             
    }
    ~LeapControl() {
        if (data_file.is_open()) {
            data_file.close();
        }
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_r;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_l;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrated_r_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrated_l_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrated_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr rpy_points_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gest_sub_r;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gest_sub_l;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr palm_sub;
    geometry_msgs::msg::Pose palm_data;
    px4_msgs::msg::VehicleLocalPosition pos_msg;

    std_msgs::msg::Bool arm_message;
    geometry_msgs::msg::Twist twist;
    std::string gest_r;
    std::string gest_l;
    bool calibrated_r;
    bool calibrated_l;
    bool reset;
    bool calibrated;
    bool mission_flag;
    int eval_flag;

    std::string current_mode;
    std::chrono::_V2::steady_clock::time_point start;

    char key;
    std::unordered_map<std::string,std::tuple<int,int,int,int>> lin_coord_map;

    std::ofstream data_file;
    int read_gest_counter;
    
    float speed;
    float turn_angle;
    int init_flag;
    int msg_ctr;
    int eval_stage;
    int start_timer;
    int final_time;

    float x;
    float y;
    float z;
    float w;

    float x_val;
    float y_val;
    float z_val;
    float w_val;

    std_msgs::msg::Bool arm_state;

    std::string drone_command;

    std::string current_gesture_r;
    std::string current_gesture_l;

    geometry_msgs::msg::Vector3 rpy;

    rclcpp::TimerBase::SharedPtr gesture_timer;
    rclcpp::TimerBase::SharedPtr speed_timer;
    geometry_msgs::msg::PoseArray pose_r;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;

    control_state current_state;


    void carth_control()
    {
        float z = pose_r.poses.at(0).position.y;
        float y = pose_r.poses.at(0).position.x;
        float x = pose_r.poses.at(0).position.z;
        std::string command = "UNKNOWN";
        if(gest_r == "FIVE")
        {
            Direction dir = get_carth_direction(x,y,palm_data.position.z,palm_data.position.x,0.025);

            switch(dir)
            {
                case FORWARD:
                    command = "FORWARD";
                    break;
                case BACKWARD:
                    command = "BACKWARD";
                    break;
                case RIGHT:
                    command = "RIGHT";
                    break;
                case LEFT:
                    command = "LEFT";
                    break;
            }

            if(z >= 0.25)
            {
                command = "UP";
            }
            else if (z < 0.15)
            {
                command = "DOWN";
            }

        }
        else if(gest_r == "THUMB")
        {
            if((rpy.x >= 0 && rpy.x <= 20) && (rpy.y >= -50 && rpy.y <= 5))
            {
                command = "TURN_L";
            }
            else if((rpy.x >= 0 && rpy.x <= 20) && rpy.y <= -100)
            {
                command = "TURN_R";
            }
        }
        drone_command = command;
    }

    void rpy_control()
    {
        std::string command = "UNKNOWN";
        float z = pose_r.poses.at(0).position.y;
        speed = 1;

        if(gest_r == "FIVE")
        {
            if(rpy.x >= 25)
            {
                command = "RIGHT";
                if(rpy.x > 25)
                {
                    speed = 1.0 + ((rpy.x) / (25.0 * 2.5));
                    speed = std::min(speed, 2.0f);
                    
                }
                else
                {
                    speed = 1;
                }
            }
            else if(rpy.x <= -25)
            {
                command = "LEFT";
                if(rpy.x < -25)
                {
                    speed = 1.0 + ((rpy.x*-1) / (25.0 * 2.5));
                    speed = std::min(speed, 2.0f);
                }
                else
                {
                    speed = 1;
                }
            }
            else if(rpy.y >= 20)
            {
                command = "BACKWARD";
                if(rpy.y > 20)
                {
                    speed = 1.0 + ((rpy.y) / (20.0 * 2.5));
                    speed = std::min(speed, 2.0f);
                }
                else
                {
                    speed = 1;
                }
            }
            else if(rpy.y <= -20)
            {
                command = "FORWARD";
                if(rpy.y < -20)
                {
                    speed = 1.0 + ((rpy.y*-1) / (20.0 * 2.5));
                    speed = std::min(speed, 2.0f);
                }
                else
                {
                    speed = 1;
                }
            }

            if(z >= 0.2)
            {
                command = "UP";
            }
            else if (z < 0.1)
            {
                command = "DOWN";
            }

        }
        else if(gest_r == "THUMB")
        {
            if((rpy.x >= 0 && rpy.x <= 20) && (rpy.y >= -50 && rpy.y <= 5))
            {
                command = "TURN_L";
            }
            else if((rpy.x >= 0 && rpy.x <= 20) && rpy.y <= -100)
            {
                command = "TURN_R";
            }
        }

        drone_command = command;
    }
    Direction get_carth_direction(float x, float y, float calibratedX, float calibratedY, float neutralRadius) 
    {
        float deltaX = x - calibratedX;
        float deltaY = y - calibratedY;
        float distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

        // Check if within neutral zone
        if (distance <= neutralRadius) {
            return Direction::NEUTRAL;
        }

        float angle = std::atan2(deltaY, deltaX);
        if (angle < 0) {
            angle += 2 * M_PI; // Ensure angle is within [0, 2π)
        }

        // Determine direction based on angle
        if (angle >= M_PI / 4 && angle < 3 * M_PI / 4) {
            return Direction::RIGHT;
        } else if (angle >= 3 * M_PI / 4 && angle < 5 * M_PI / 4) {
            return Direction::FORWARD;
        } else if (angle >= 5 * M_PI / 4 && angle < 7 * M_PI / 4) {
            return Direction::LEFT;
        } else {
            return Direction::BACKWARD;
        }
    }

    void pose_callback_r(geometry_msgs::msg::PoseArray msg)
    {
        pose_r = msg;
    }

    void get_rpy(geometry_msgs::msg::Vector3 msg)
    {
        rpy.x = msg.x;
        rpy.y = msg.y;
        rpy.z = msg.z;
    }

    void get_gest_r(std_msgs::msg::String msg)
    {
        gest_r = msg.data;
    }
    void get_gest_l(std_msgs::msg::String msg)
    {
        gest_l = msg.data;
    }

    void write_data_to_file(std::string hand_type, std::string desired_gest, std::string current_gest)
    {   
        this->data_file << hand_type << ' ';
        this->data_file << desired_gest << ' ';
        this->data_file << current_gest << ' ';
        this->data_file << gesture_check(desired_gest, current_gest);

        this->data_file << std::endl;
    }
    int gesture_check(std::string desired_gest, std::string current_gest)
    {
        if(current_gest == desired_gest)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    void palm_callback(geometry_msgs::msg::Pose msg)
    {
        palm_data.position.x = msg.position.x;
        palm_data.position.y = msg.position.y;
        palm_data.position.z = msg.position.z;
    }

    void mode_switch(std::string gesture_l)
    {
        if(gesture_l == "ONE")
        {
            current_mode = "MODE_1";
            speed = 1;
        }
        else if(gesture_l == "TWO")
        {
            current_mode = "MODE_2";
            speed = 1;
        }
        else
        {
            current_mode = current_mode;
        }
    }
    void calibrated_r_callback(std_msgs::msg::Bool msg)
        {
            calibrated_r = msg.data;
        }
    void calibrated_l_callback(std_msgs::msg::Bool msg)
        {
            calibrated_l = msg.data;
        }
    void reset_callback(std_msgs::msg::Bool msg)
        {
            reset = msg.data;
        }
    void calibrated_callback(std_msgs::msg::Bool msg)
        {
            calibrated = msg.data;
        }
    void collect_gesture_data(std::string desired_r_gesture, std::string desired_l_gesture)                          
    {   

        std::cout<<"STARTING GESTURE DATA COLLECTION" << std::endl;
        start = steady_clock::now();

        if((steady_clock::now() - start) <= seconds(2))
        {
            write_data_to_file("R",desired_r_gesture, gest_r);
            write_data_to_file("L",desired_l_gesture, gest_l);
        }
        std::cout<<"Gesture data collected" << std::endl;
    }
        
    void count_down()
    {
        auto start = steady_clock::now();
        auto last_second_mark = start;

        std::cout<<"STARTING EVALUATION IN :"<<std::endl;
                    
        while(duration_cast<seconds>(steady_clock::now() - start) <= seconds(3))
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

    void vehicle_position_callback(px4_msgs::msg::VehicleLocalPosition msg){
        pos_msg = msg;
    }
        
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto leap_control = std::make_shared<LeapControl>();

    rclcpp::spin(leap_control);

    rclcpp::shutdown();
    return 0;
}
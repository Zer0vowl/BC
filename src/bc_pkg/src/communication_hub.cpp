#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <map>
#include <chrono>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define ENUM_TO_STRING_CASE(x) case x: return #x

class OffboardControl : public rclcpp::Node
{
    public:
    OffboardControl() : Node("offboard_control")
    {
        static const rmw_qos_profile_t qos_profile = 
        {   
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
        
        // Create subscriptions
        vehicle_status_sub = this->create_subscription
            <VehicleStatus>("/fmu/out/vehicle_status", qos,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_velocity_sub = this->create_subscription
            <geometry_msgs::msg::Twist>("/offboard_velocity_cmd", qos,
            std::bind(&OffboardControl::offboard_velocity_callback, this, std::placeholders::_1));


        vehicle_attitude_sub = this->create_subscription
            <VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
            std::bind(&OffboardControl::attitude_callback, this, std::placeholders::_1));


        vehicle_arm_sub = this->create_subscription
            <std_msgs::msg::Bool>("/arm_message", qos,
            std::bind(&OffboardControl::arm_message_callback, this, std::placeholders::_1));


        // Create publishers
        offboard_control_pub_ = this->create_publisher
            <OffboardControlMode>("/fmu/in/offboard_control_mode", qos);

        trajectory_setpoint_pub = this->create_publisher
            <TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);

        vehicle_command_pub_ = this->create_publisher
            <VehicleCommand>("/fmu/in/vehicle_command", 10);

        vehicle_velocity_pub = this->create_publisher
            <geometry_msgs::msg::Twist>("/fmu/in/setpoint_velocity/cmd_vel_unstamped", qos);


        // Create timers
        auto arm_timer_callback = [this]() -> void {

            switch(current_state){

                case IDLE:
                    if((this->flight_check && this->arm_msg) == true)
                    {
                        current_state = ARMING;
                    }
                    break;

                case ARMING:
                    if(!(this->flight_check))
                    {
                        current_state = IDLE;
                    }
                    else if((this->arm_state == VehicleStatus::ARMING_STATE_ARMED) && (count > 10))
                    {
                        current_state = TAKEOFF;
                    }
                    this->arm();
                    break;

                case TAKEOFF:
                    if(!(this->flight_check))
                    {
                        current_state = IDLE;
                    }
                    else if(this->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF){
                        current_state = LOITER;
                    }
                    this->arm();
                    this->take_off();
                    break;

                // waits in this state while taking off, and the 
                // moment VehicleStatus switches to Loiter state it will switch to offboard
                case LOITER: 
                    if(!(flight_check))
                    {
                        current_state = IDLE;
                    }
                    else if(nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER)
                    {
                        current_state = OFFBOARD;
                    }
                    this->arm();
                    break;

                case OFFBOARD:
                    if((!(flight_check)) || (arm_state == VehicleStatus::ARMING_STATE_DISARMED) || (failsafe == true))
                    {
                        current_state = IDLE;
                    }
                    this->state_offboard();
                    break;
            }

            if(arm_state != VehicleStatus::ARMING_STATE_ARMED)
            {
                arm_msg = false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), flightStateToString(current_state));
            count += 1;

        };
        arming_timer = this->create_wall_timer(100ms, arm_timer_callback);

            auto cmdloop_callback = [this]() -> void {
            if(offboard_mode == true){
                OffboardControlMode offboard_msg{};
                offboard_msg.timestamp =  this->get_clock()->now().nanoseconds() / 1000;
                offboard_msg.position = false;
                offboard_msg.velocity = true;
                offboard_msg.acceleration = false;
                offboard_control_pub_->publish(offboard_msg); 

                float cos_yaw = cos(true_yaw);
                float sin_yaw = sin(true_yaw);
                float velocity_world_x = ((velocity.x * cos_yaw) - (velocity.y * sin_yaw));
                float velocity_world_y = ((velocity.x * sin_yaw) + (velocity.y * cos_yaw));

                TrajectorySetpoint trajectory_msg{};
                trajectory_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                trajectory_msg.velocity[0] = velocity_world_x;
                trajectory_msg.velocity[1] = velocity_world_y;
                trajectory_msg.velocity[2] = velocity.z;
                trajectory_msg.position[0] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.position[1] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.position[2] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.yaw = std::numeric_limits<float>::quiet_NaN();
                trajectory_msg.yawspeed = yaw;
                
                trajectory_setpoint_pub->publish(trajectory_msg);
            }
        };

        command_timer = this->create_wall_timer(20ms, cmdloop_callback);

        // Initialize variables
        nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX;
        arm_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        velocity = geometry_msgs::msg::Vector3();
        yaw = 0.0;
        true_yaw = 0.0;
        offboard_mode = false;
        flight_check = false;
        count = 0;
        arm_msg = false;
        failsafe = false;
        current_state = IDLE;

        // States with corresponding callback functions that run once when state switches
        state_map[IDLE] = std::bind(&OffboardControl::state_init, this);
        state_map[ARMING] = std::bind(&OffboardControl::state_arming, this);
        state_map[TAKEOFF] = std::bind(&OffboardControl::state_takeoff, this);
        state_map[LOITER] = std::bind(&OffboardControl::state_loiter, this);
        state_map[OFFBOARD] = std::bind(&OffboardControl::state_offboard, this);
    }

private:
    //subscribers
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vehicle_velocity_sub;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vehicle_arm_sub;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;

    //publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_pub_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vehicle_velocity_pub;
    
    void arm();
    void take_off();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0);

    //states
    using state = std::function<void()>;

    enum flight_states{
            IDLE,
            ARMING,
            TAKEOFF,
            LOITER,
            OFFBOARD
            };

    const char* flightStateToString(flight_states state) {
        switch (state) {
            ENUM_TO_STRING_CASE(IDLE);
            ENUM_TO_STRING_CASE(ARMING);
            ENUM_TO_STRING_CASE(TAKEOFF);
            ENUM_TO_STRING_CASE(LOITER);
            ENUM_TO_STRING_CASE(OFFBOARD);
            default: return "UNKNOWN_STATE";
            }
    }
    std::unordered_map<flight_states,state> state_map;

    flight_states current_state;

    rclcpp::TimerBase::SharedPtr arming_timer;
    rclcpp::TimerBase::SharedPtr command_timer;

    uint8_t nav_state;
    uint8_t arm_state;
    geometry_msgs::msg::Vector3 velocity;

    float yaw;
    float true_yaw;

    bool offboard_mode;
    bool flight_check;
    bool arm_msg;
    bool failsafe;

    int count;


    void state_init()
    {
        count = 0;
    }

    void state_arming()
    {
        count = 0;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(get_logger(), "Arm command send");
    }

    void state_takeoff()
    {
        count = 0;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0, 0.0, 5.0);
        RCLCPP_INFO(get_logger(), "Takeoff command send");
    }

    void state_loiter()
    {
        count = 0;
        RCLCPP_INFO(get_logger(), "Loiter Status");
    }

    void state_offboard()
    {
        count = 0;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        offboard_mode = true;
    }

    void arm_message_callback(std_msgs::msg::Bool msg){
        arm_msg = msg.data;
    }

    void vehicle_status_callback(VehicleStatus msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NAV STATUS: %d", msg.nav_state);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ARM STATUS: %d", msg.arming_state);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ARM STATUS: %d", count);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failsafe: %d", msg.failsafe);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Flight Check: %d", msg.pre_flight_checks_pass);

        nav_state = msg.nav_state;
        arm_state = msg.arming_state;
        failsafe = msg.failsafe;
        flight_check = msg.pre_flight_checks_pass;
    }
    void offboard_velocity_callback(geometry_msgs::msg::Twist msg){

        velocity.x = -msg.linear.y;
        velocity.y = msg.linear.x;
        velocity.z = -msg.linear.z;

        yaw = msg.angular.z;

        std::cout<<std::to_string(velocity.x)<<"X"<<std::endl;
        std::cout<<std::to_string(velocity.y)<<"Y"<<std::endl;
        std::cout<<std::to_string(velocity.z)<<"Z"<<std::endl;
        std::cout<<std::to_string(yaw)<<"W"<<std::endl;
    }
    void attitude_callback(px4_msgs::msg::VehicleAttitude msg){

        auto orientation_q = msg.q;

        true_yaw = -(atan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                     1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])));
    
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "YAW: %f", true_yaw);
    }
};


    void OffboardControl::arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arm command send");
    }

    void OffboardControl::take_off()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0, 0.0, 5.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff command send");
    }

    void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param7)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param7 = param7;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;


        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TP1 %f", msg.param1);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TP2 %f", msg.param2);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TP3 %f", msg.param3);


        vehicle_command_pub_->publish(msg);
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto offboard_control = std::make_shared<OffboardControl>();

    rclcpp::spin(offboard_control);

    rclcpp::shutdown();
    return 0;
}
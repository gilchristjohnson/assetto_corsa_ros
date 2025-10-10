// Node to transfer AWSIM messages into what we're expecting from the raptor

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <functional>
#include <mutex>


#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "autonoma_msgs/msg/vehicle_inputs.hpp"
#include "autonoma_msgs/msg/vehicle_data.hpp"
#include "autonoma_msgs/msg/powertrain_data.hpp"
#include "autonoma_msgs/msg/race_control.hpp"
#include "autonoma_msgs/msg/to_raptor.hpp"

#include "raptor_dbw_msgs/msg/pt_report1.hpp"
#include "raptor_dbw_msgs/msg/pt_report3.hpp"
#include "raptor_dbw_msgs/msg/wheel_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_report_struct.hpp"
#include "raptor_dbw_msgs/msg/marelli_report1.hpp"
#include "raptor_dbw_msgs/msg/misc_report.hpp"
#include "raptor_dbw_msgs/msg/ct_report.hpp"
#include "raptor_dbw_msgs/msg/steering_report_extd.hpp"
#include "raptor_dbw_msgs/msg/brake_pressure_report.hpp"

#include "iac_msgs/msg/car_cmd.hpp"
#include "teleop_msgs/msg/float32_stamped.hpp"

#include "iac_msgs/iac_qos.hpp"

class ros2awsim : public rclcpp::Node
{
public:
    ros2awsim()
        : Node("ros2awsim_node")
    {
        // Node settings

        // Parameters
        this->declare_parameter("version", GIT_INFO);
        this->set_parameters({rclcpp::Parameter("version", GIT_INFO)});

        this->declare_parameter<int>("car", 8);
        veh_num_ = this->get_parameter("car").as_int();

        this->declare_parameter("sim_flags", true);
        sim_flags_ = this->get_parameter("sim_flags").as_bool();

        // Initializing wheel report struct
        for (int ii = 0; ii < 16; ii++) {
            msg_WheelReportStruct_.front_left.temperature.push_back(-1.0f);
            msg_WheelReportStruct_.front_right.temperature.push_back(-1.0f);
            msg_WheelReportStruct_.rear_left.temperature.push_back(-1.0f);
            msg_WheelReportStruct_.rear_right.temperature.push_back(-1.0f);
        }

        // Publishers
        pub_VehicleInputs_ = this->create_publisher<autonoma_msgs::msg::VehicleInputs>("/vehicle_inputs", 1);
        pub_ToRaptor_ = this->create_publisher<autonoma_msgs::msg::ToRaptor>("/to_raptor",1);
        pub_Wheel_ = this->create_publisher<raptor_dbw_msgs::msg::WheelReportStruct>("/raptor/sensor/wheel/all",qos_raptor());
        pub_PtReport1_ = this->create_publisher<raptor_dbw_msgs::msg::PtReport1>("/raptor/powertrain/report_1",qos_raptor());
        pub_PtReport3_ = this->create_publisher<raptor_dbw_msgs::msg::PtReport3>("/raptor/powertrain/report_3",qos_raptor());
        pub_MiscReport_ = this->create_publisher<raptor_dbw_msgs::msg::MiscReport>("/raptor/diagnostic/misc_report",qos_raptor());
        pub_SteeringReportExtd_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringReportExtd>("/raptor/sensor/steering/report_extd",qos_raptor());
        pub_BrakePressureReport_ = this->create_publisher<raptor_dbw_msgs::msg::BrakePressureReport>("/raptor/sensor/brake/pressure_report",qos_raptor());
        
        pub_IncremementStateMachine_ = this->create_publisher<std_msgs::msg::Empty>("/raptor/state/cmd/sim/start",1);

        if (sim_flags_)
            pub_RaceFlag_ = this->create_publisher<raptor_dbw_msgs::msg::MarelliReport1>("/raptor/sensor/marelli/report_1",qos_raptor());

        // Subscribers
        sub_CarCmd_ = this->create_subscription<iac_msgs::msg::CarCmd>(
            "/raptor/cmd",
            qos_raptor(),
            std::bind(&ros2awsim::callback_CarCmd, this, std::placeholders::_1)
        );
        subCtReport_ = this->create_subscription<raptor_dbw_msgs::msg::CtReport>(
            "/raptor/ct_report",
            qos_raptor(),
            std::bind(&ros2awsim::callback_CtReport, this, std::placeholders::_1)
        );

        subPowertrainData_ = this->create_subscription<autonoma_msgs::msg::PowertrainData>(
            "/powertrain_data",
            1,
            std::bind(&ros2awsim::callback_PowerTrainData, this, std::placeholders::_1)
        );
        subVehicleData_ = this->create_subscription<autonoma_msgs::msg::VehicleData>(
            "/vehicle_data",
            1,
            std::bind(&ros2awsim::callback_VehicleData, this, std::placeholders::_1)
        );

        if (sim_flags_)
            subRaceControl_ = this->create_subscription<autonoma_msgs::msg::RaceControl>(
                "/race_control",
                1,
                std::bind(&ros2awsim::callback_RaceControl, this, std::placeholders::_1)
            );

        
        // Artificial control disturbance subscribers
        subThrottleDisturbance_ = this->create_subscription<teleop_msgs::msg::Float32Stamped>(
            "/vehicle_inputs/sim/throttle_disturbance",
            1,
            std::bind(&ros2awsim::callback_ThrottleDisturbance, this, std::placeholders::_1));
        subSteeringDisturbance_ = this->create_subscription<teleop_msgs::msg::Float32Stamped>(
            "/vehicle_inputs/sim/steering_disturbance",
            1,
            std::bind(&ros2awsim::callback_SteeringDisturbance, this, std::placeholders::_1));

        // Timers
        timer_1Hz_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ros2awsim::callback_1Hz, this)
        ); 

        // Printouts
        RCLCPP_INFO(this->get_logger(), "ROS2 to AWSIM");

        // Initialisation done
        return;
    }

private:
    // Subscribers / Publishers / Timers
    rclcpp::Subscription<iac_msgs::msg::CarCmd>::SharedPtr sub_CarCmd_;
    rclcpp::Subscription<raptor_dbw_msgs::msg::CtReport>::SharedPtr subCtReport_;
    rclcpp::Subscription<autonoma_msgs::msg::VehicleData>::SharedPtr subVehicleData_;
    rclcpp::Subscription<autonoma_msgs::msg::PowertrainData>::SharedPtr subPowertrainData_;
    rclcpp::Subscription<autonoma_msgs::msg::RaceControl>::SharedPtr subRaceControl_;
    rclcpp::Subscription<teleop_msgs::msg::Float32Stamped>::SharedPtr subThrottleDisturbance_;
    rclcpp::Subscription<teleop_msgs::msg::Float32Stamped>::SharedPtr subSteeringDisturbance_;

    rclcpp::Publisher<autonoma_msgs::msg::VehicleInputs>::SharedPtr pub_VehicleInputs_;
    rclcpp::Publisher<autonoma_msgs::msg::ToRaptor>::SharedPtr pub_ToRaptor_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::WheelReportStruct>::SharedPtr pub_Wheel_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::PtReport1>::SharedPtr pub_PtReport1_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::PtReport3>::SharedPtr pub_PtReport3_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::MarelliReport1>::SharedPtr pub_RaceFlag_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::MiscReport>::SharedPtr pub_MiscReport_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReportExtd>::SharedPtr pub_SteeringReportExtd_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::BrakePressureReport>::SharedPtr pub_BrakePressureReport_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_IncremementStateMachine_;

    rclcpp::TimerBase::SharedPtr timer_1Hz_;

    // Variables
    autonoma_msgs::msg::VehicleInputs msg_VehicleInputs_;
    autonoma_msgs::msg::ToRaptor msg_ToRaptor_;
    raptor_dbw_msgs::msg::WheelReportStruct msg_WheelReportStruct_;
    raptor_dbw_msgs::msg::PtReport1 msg_PtReport1_;
    raptor_dbw_msgs::msg::PtReport3 msg_PtReport3_;
    raptor_dbw_msgs::msg::MarelliReport1 msg_MarelliReport1_;
    raptor_dbw_msgs::msg::MiscReport msg_MiscReport_;

    float throttle_disturbance = 0.0f;
    float steering_disturbance = 0.0f;
    float steering_multiplier_ = 1.02*2.4f*17.0f/16.0f; // Patrick's circle testing

    bool sim_flags_ = 1;

    uint8_t veh_num_ = 1;

    void callback_PowerTrainData(const autonoma_msgs::msg::PowertrainData::SharedPtr msg)
    {

        // Convert the PtReport message into a PowertrainData message

        // PtReport1
        msg_PtReport1_.header = msg->header;
        msg_PtReport1_.engine_run_switch = msg->engine_run_switch_status;
        msg_PtReport1_.engine_state = msg->engine_on_status; // Might be wrong
        msg_PtReport1_.throttle_position = msg->throttle_position;
        msg_PtReport1_.current_gear = msg->current_gear;
        msg_PtReport1_.engine_speed_rpm = msg->engine_rpm;
        msg_PtReport1_.vehicle_speed_kmph = msg->vehicle_speed_kmph;
        msg_PtReport1_.gear_shift_status = raptor_dbw_msgs::msg::PtReport1::GEAR_SHIFT_STATUS_GEAR_SHIFT_READY; //msg->gear_shift_status;
        pub_PtReport1_->publish(msg_PtReport1_);

        // PtReport3
        msg_PtReport3_.header = msg->header;
        msg_PtReport3_.engine_oil_temperature = msg->engine_oil_temperature;
        msg_PtReport3_.torque_wheels = msg->torque_wheels_nm;
        pub_PtReport3_->publish(msg_PtReport3_);

        // All done
        return;
    }

    void callback_VehicleData(const autonoma_msgs::msg::VehicleData::SharedPtr msg)
    {

        // Publish WheelReport
        msg_WheelReportStruct_.header = msg->header;

        msg_WheelReportStruct_.front_left.pressure = msg->fl_tire_pressure;
        msg_WheelReportStruct_.front_left.pressure_guage = msg->fl_tire_pressure_gauge;
        msg_WheelReportStruct_.front_left.strain_guage = msg->fl_wheel_load;
        msg_WheelReportStruct_.front_right.pressure = msg->fr_tire_pressure;
        msg_WheelReportStruct_.front_right.pressure_guage = msg->fr_tire_pressure_gauge;
        msg_WheelReportStruct_.front_right.strain_guage = msg->fr_wheel_load;
        msg_WheelReportStruct_.rear_left.pressure = msg->rl_tire_pressure;
        msg_WheelReportStruct_.rear_left.pressure_guage = msg->rl_tire_pressure_gauge;
        msg_WheelReportStruct_.rear_left.strain_guage = msg->rl_wheel_load;
        msg_WheelReportStruct_.rear_right.pressure =msg->rr_tire_pressure;
        msg_WheelReportStruct_.rear_right.pressure_guage =msg->rr_tire_pressure_gauge;
        msg_WheelReportStruct_.rear_right.strain_guage = msg->rr_wheel_load;
    
        msg_WheelReportStruct_.front_left.speed = msg->ws_front_left/3.6;
        msg_WheelReportStruct_.front_right.speed = msg->ws_front_right/3.6;
        msg_WheelReportStruct_.rear_left.speed = msg->ws_rear_left/3.6;
        msg_WheelReportStruct_.rear_right.speed = msg->ws_rear_right/3.6;

        for (int ii = 0; ii < 16; ii++)
        {
            msg_WheelReportStruct_.front_left.temperature[ii] = msg->fl_tire_temperature;
            msg_WheelReportStruct_.front_right.temperature[ii] = msg->fr_tire_temperature;
            msg_WheelReportStruct_.rear_left.temperature[ii] = msg->rl_tire_temperature;
            msg_WheelReportStruct_.rear_right.temperature[ii] = msg->rr_tire_temperature;
        }

        pub_Wheel_->publish(msg_WheelReportStruct_);

        // Publish MiscReport
        msg_MiscReport_.header = msg->header;
        msg_MiscReport_.battery_voltage = msg->battery_voltage;
        msg_MiscReport_.sys_state = msg->sys_state;
        msg_MiscReport_.mode_switch_state = raptor_dbw_msgs::msg::MiscReport::MODE_STATE_SWITCH_RACE_MODE;

        pub_MiscReport_->publish(msg_MiscReport_);

        // Publish SteeringReportExtd
        raptor_dbw_msgs::msg::SteeringReportExtd msg_SteeringReportExtd;
        msg_SteeringReportExtd.header = msg->header;
        msg_SteeringReportExtd.primary_steering_angle_fbk = msg->steering_wheel_angle/steering_multiplier_;
        msg_SteeringReportExtd.secondary_steering_ang_fdbk = msg->steering_wheel_angle/steering_multiplier_;
        msg_SteeringReportExtd.average_steering_ang_fdbk = msg->steering_wheel_angle/steering_multiplier_;
        pub_SteeringReportExtd_->publish(msg_SteeringReportExtd);

        // Publish BrakePresureReport
        raptor_dbw_msgs::msg::BrakePressureReport msg_BrakePresureReport;
        msg_BrakePresureReport.header = msg->header;
        msg_BrakePresureReport.brake_pressure_fdbk_front = msg->front_brake_pressure;
        msg_BrakePresureReport.brake_pressure_fdbk_rear = msg->rear_brake_pressure;
        pub_BrakePressureReport_->publish(msg_BrakePresureReport);
 
    }

    void callback_CtReport(const raptor_dbw_msgs::msg::CtReport::SharedPtr msg)
    {
        // Publish ToRaptor
        msg_ToRaptor_.header = msg->header;
        msg_ToRaptor_.veh_sig_ack = msg->veh_sig_ack;
        msg_ToRaptor_.track_cond_ack = msg->track_cond_ack;
        msg_ToRaptor_.ct_state = msg->ct_state;
        msg_ToRaptor_.rolling_counter = msg->ct_state_rolling_counter;
        msg_ToRaptor_.veh_num = veh_num_;
        pub_ToRaptor_->publish(msg_ToRaptor_);

    }

    void callback_CarCmd(const iac_msgs::msg::CarCmd::SharedPtr msg)
    {
        msg_VehicleInputs_.header = msg->header;
        msg_VehicleInputs_.header.stamp = this->get_clock()->now();

        msg_VehicleInputs_.throttle_cmd = (msg->acc_pedal_cmd) + throttle_disturbance;
        msg_VehicleInputs_.brake_f_cmd = msg->f_brake_pressure_cmd;
        msg_VehicleInputs_.brake_r_cmd = msg->r_brake_pressure_cmd;
        msg_VehicleInputs_.steering_cmd = steering_multiplier_*(msg->steering_motor_ang_cmd) + steering_disturbance;
        msg_VehicleInputs_.gear_cmd = msg->desired_gear;

        // Fill out counters
        msg_VehicleInputs_.throttle_cmd_count = msg_ToRaptor_.rolling_counter;
        msg_VehicleInputs_.brake_f_cmd_count    = msg_ToRaptor_.rolling_counter;
        msg_VehicleInputs_.brake_r_cmd_count    = msg_ToRaptor_.rolling_counter;
        msg_VehicleInputs_.steering_cmd_count = msg_ToRaptor_.rolling_counter;

        // Publish
        pub_VehicleInputs_->publish(msg_VehicleInputs_);

    }

    void callback_RaceControl(const autonoma_msgs::msg::RaceControl::SharedPtr msg)
    {
        // Convert the RaceControl message into a MarelliReport1 message
        msg_MarelliReport1_.header = msg->header;

        msg_MarelliReport1_.marelli_track_flag = msg->track_flag;
        msg_MarelliReport1_.marelli_vehicle_flag = msg->veh_flag;

        if (sim_flags_)
            pub_RaceFlag_->publish(msg_MarelliReport1_);

        msg_MarelliReport1_.marelli_rc_base_sync_check = true;
        msg_MarelliReport1_.marelli_rc_lte_rssi = 50;
        msg_MiscReport_.raptor_rolling_counter = msg->base_to_car_heartbeat;

    }

    void callback_1Hz()
    {
        // If not in drive mode, increment the state machine
        if (msg_MiscReport_.sys_state != raptor_dbw_msgs::msg::MiscReport::SYSTEM_STATE_DRIVING)
        {
            RCLCPP_INFO(this->get_logger(), "Incrementing sim state machine");
            std_msgs::msg::Empty msg;
            pub_IncremementStateMachine_->publish(msg);
        }

        // All done
        return;
    }

    void callback_ThrottleDisturbance(const teleop_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        throttle_disturbance = msg->data;
    }
    void callback_SteeringDisturbance(const teleop_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        steering_disturbance = msg->data;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2awsim>());
    rclcpp::shutdown();
    return 0;
}

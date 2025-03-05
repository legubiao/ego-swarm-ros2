//
// Created by tlab-uav on 25-3-5.
//

#ifndef OFFBOARDCONTROL_H
#define OFFBOARDCONTROL_H
#include <px4_msgs/msg/detail/offboard_control_mode__struct.hpp>
#include <px4_msgs/msg/detail/trajectory_setpoint__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <rclcpp/rclcpp.hpp>


class OffBoardControl {
public:
    OffBoardControl(rclcpp::Node::SharedPtr node);

protected:
    rclcpp::Node::SharedPtr node_;

    void arm() const;
    void disarm() const;
    void publishOffBoardControlMode();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
};



#endif //OFFBOARDCONTROL_H

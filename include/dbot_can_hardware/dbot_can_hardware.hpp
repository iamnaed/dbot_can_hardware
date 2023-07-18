#ifndef DBOT_CAN_HARDWARE__DBOT_CAN_HARDWARE_HPP_
#define DBOT_CAN_HARDWARE__DBOT_CAN_HARDWARE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dbot_can_hardware/visibility_control.h"
#include "dbot_can_hardware/dbot_can.hpp"

namespace dbot_can_hardware
{

    class DbotCanHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DbotCanHardware)

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_CAN_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        DBOT_CAN_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        DBOT_CAN_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Parameters for Dbot Can Hardware
        dbot_can::DbotCan dbot_can_;

        // Store commands
        std::vector<double> cmd_positions_;
        std::vector<double> state_positions_;
        std::vector<double> state_velocities_;
        std::vector<double> state_accelerations_;
        
        // Storage
        std::vector<double> state_positions_previous_;
    };
}


#endif  // DBOT_CAN_HARDWARE__DBOT_CAN_HARDWARE_HPP_
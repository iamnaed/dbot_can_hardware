#include <chrono>
#include <memory>
#include <vector>

#include "dbot_can_hardware/dbot_can_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dbot_can_hardware
{
    // Initialize
    hardware_interface::CallbackReturn DbotCanHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Initializing DbotCan...");

        // Set config 
        // Get data from the hardware parameter
        dbot_can::DbotCanConfig config;
        config.can_name = info_.hardware_parameters["can_name"];

        // Hex string numbers to int
        config.joint_can_ids[0] = std::stoi(info_.hardware_parameters["can_id_j0"],0,16);
        config.joint_can_ids[1] = std::stoi(info_.hardware_parameters["can_id_j1"],0,16);
        config.joint_can_ids[2] = std::stoi(info_.hardware_parameters["can_id_j2"],0,16);
        config.joint_can_ids[3] = std::stoi(info_.hardware_parameters["can_id_j3"],0,16);
        config.joint_can_ids[4] = std::stoi(info_.hardware_parameters["can_id_j4"],0,16);
        config.joint_can_ids[5] = std::stoi(info_.hardware_parameters["can_id_j5"],0,16);
        config.joint_can_ids[0] = 0x001;
        config.joint_can_ids[1] = 0x002;
        config.joint_can_ids[2] = 0x003;
        config.joint_can_ids[3] = 0x004;
        config.joint_can_ids[4] = 0x005;
        config.joint_can_ids[5] = 0x006;

        // Float strings to float
        config.joint_reduction_ratios[0] = std::stof(info_.hardware_parameters["redc_ratio_j0"]);
        config.joint_reduction_ratios[1] = std::stof(info_.hardware_parameters["redc_ratio_j1"]);
        config.joint_reduction_ratios[2] = std::stof(info_.hardware_parameters["redc_ratio_j2"]);
        config.joint_reduction_ratios[3] = std::stof(info_.hardware_parameters["redc_ratio_j3"]);
        config.joint_reduction_ratios[4] = std::stof(info_.hardware_parameters["redc_ratio_j4"]);
        config.joint_reduction_ratios[5] = std::stof(info_.hardware_parameters["redc_ratio_j5"]);
        config.joint_reduction_ratios[0] = 48;
        config.joint_reduction_ratios[1] = 48;
        config.joint_reduction_ratios[2] = 48;
        config.joint_reduction_ratios[3] = 48;
        config.joint_reduction_ratios[4] = 48;
        config.joint_reduction_ratios[5] = 48;

        // Initialize dbot_can_
        dbot_can_.initialize(config);

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "DbotCan initialization complete...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Establish communication
    hardware_interface::CallbackReturn DbotCanHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        // Error suppress
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Establishing communications to motor controllers...");

        // // Connect
        // bool suc = dbot_can_.connect();
        // if(!suc)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("DbotCanHardware"), "Communication failed to open for some reason. . .");
        //     return hardware_interface::CallbackReturn::FAILURE;
        // }

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Communication opened successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Disconnect communications
    hardware_interface::CallbackReturn DbotCanHardware::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        // Error suppress
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Disconnecting communications to motor controllers...");
        
        // // Connect
        // bool suc = dbot_can_.disconnect();
        // if(!suc)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("DbotCanHardware"), "Disconnecting failed for some reason. . .");
        //     return hardware_interface::CallbackReturn::FAILURE;
        // }

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Communication disconected successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Engage actuators
    hardware_interface::CallbackReturn DbotCanHardware::on_activate(const rclcpp_lifecycle::State& previous_state) 
    {
        // Error suppress
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Engaging motors...");
        
        // // Attempt to start the motors
        // bool suc = dbot_can_.engage_motor();
        // if(!suc)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("DbotCanHardware"), "Engaging motors failed for some reason. . .");
        //     return hardware_interface::CallbackReturn::FAILURE;
        // }

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Motors engaged successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Disengage actuators
    hardware_interface::CallbackReturn DbotCanHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) 
    {
        // Error suppress
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Disengaging motors...");
        
        // // Attempt to stop the motors
        // bool suc = dbot_can_.disengage_motor();
        // if(!suc)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("DbotCanHardware"), "Disengaging motors failed for some reason. . .");
        //     return hardware_interface::CallbackReturn::FAILURE;
        // }

        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Motors disengaged successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Cleanup
    hardware_interface::CallbackReturn DbotCanHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state) 
    {
        // Error suppress
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Shutting down.. please wait...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Shut down successfull. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Export state interfaces
    std::vector<hardware_interface::StateInterface> DbotCanHardware::export_state_interfaces() 
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // State Position
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_positions_[i]));

            // State Velocity
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]));
        }
        return state_interfaces;
    }

    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> DbotCanHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // Position command interface
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
        }
        return command_interfaces;
    }

    // Read data
    hardware_interface::return_type DbotCanHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Error suppress
        (void)period;
        (void)time;

        // // Positions and velocities
        // std::array<float, 6> pos = dbot_can_.get_position();
        // std::array<float, 6> vel = dbot_can_.get_velocity();

        // // There is a hidden conversion from float to double
        // auto len = pos.size();
        // for (auto i = 0u; i < len; i++)
        // {
        //     // Set Position
        //     state_positions_[i] = pos[i];

        //     // Set Velocity
        //     state_velocities_[i] = vel[i];
        // }

        // Positions and velocities
        auto len = info_.joints.size();
        for (auto i = 0u; i < len; i++)
        {
            // Previous position
            double prev_pos = state_positions_[i];
            double prev_vel = state_velocities_[i];
            double deltaSeconds = period.seconds();

            // Set Position
            state_positions_[i] = cmd_positions_[i];

            // Set Velocity
            double ds = state_positions_[i] - prev_pos;
            state_velocities_[i] = ds / deltaSeconds;
        }
        return hardware_interface::return_type::OK;
    }

    // Write data
    hardware_interface::return_type DbotCanHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Error suppress
        (void)time;
        (void)period;

        // // Convert first before sending
        // std::array<float, 6> pos;
        // auto len = pos.size();
        // for (auto i = 0u; i < len; i++)
        // {
        //     // Conversion from double to float
        //     pos[i] = cmd_positions_[i];
        // }

        // // Write the data
        // dbot_can_.set_position(pos);

        //RCLCPP_INFO(rclcpp::get_logger("DbotCanHardware"), "Sending command. . .Time: %.1fs", period.seconds());
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dbot_can_hardware::DbotCanHardware, hardware_interface::SystemInterface)
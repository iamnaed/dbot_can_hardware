#ifndef DBOT_CAN__DBOT_CAN_HPP_
#define DBOT_CAN__DBOT_CAN_HPP_

#include "odrive_can.hpp"

#include <cstring>
#include <string>
#include <vector>
#include <memory>

namespace dbot_can
{
    struct DbotCanConfig
    {
        std::string can_name;
        std::array<int, 6> joint_can_ids;
        std::array<float, 6> joint_reduction_ratios;
    };

    class Joint
    {
        static const int J0 = 0;
        static const int J1 = 1;
        static const int J2 = 2;
        static const int J3 = 3;
        static const int J4 = 4;
        static const int J5 = 5;
    };

    class DbotCan
    {
    public:
        /**
         * @brief Construct a new Dbot Can object
         * 
         */
        //DbotCan();

        /**
         * @brief Construct a new Dbot Can object
         * 
         * @param config 
         */
        DbotCan(const DbotCanConfig& config);

        /**
         * @brief Initializes the Can Bus using linux's built in SocketCan
         * 
         */
        bool initialize(const DbotCanConfig& config);

        /**
         * @brief Creates a connection to the CAN bus 
         * 
         * @return true if successful, false otherwiseケビンを待っています
         */
         bool connect();

        /**
         * @brief Disconnects from the CAN bus
         * 
         * @return true if successful, false otherwise
         */
        bool disconnect();

        /**
         * @brief Engages the motor from idle to closed loop control
         * 
         * @return true if successful, false otherwise
         */
        bool engage_motor();

        /**
         * @brief Disengages the motor from closed loop control to idle
         * 
         * @return true if successful, false otherwise
         */
        bool disengage_motor();

        /**
         * @brief Get the joint positions
         * 
         * @return std::array<float, 6>
         */
        std::array<float, 6> get_position();

        /**
         * @brief Get the joint velocities
         * 
         * @return std::array<float, 6> 
         */
        std::array<float, 6> get_velocity();

        /**
         * @brief Set the joint positions
         * 
         * @param pos 
         * @return true if successful, false otherwise
         */
        bool set_position(std::array<float, 6> pos);
        
        /**
         * @brief Get errors in the odrive controllers
         * 
         * @return int 
         */
        int get_errors();

        /**
         * @brief Clear errors in the odrive controllers
         * 
         * @return true if successful, false otherwise
         */
        bool clear_errors();

    private:
        /**
         * @brief Internal members
         * 
         */
        std::string can_name_;
        std::array<int, 6> joint_can_ids_;
        std::array<float, 6> joint_reduction_ratios_;
        std::array<float, 6> joint_reduction_ratios_inverse_;
        std::array<float, 6> encoder_positions_;
        std::array<float, 6> encoder_velocities_;
        std::array<float, 6> joint_positions_;
        std::array<float, 6> joint_velocities_;

        /**
         * @brief Socket handles for CAN communication
         * 
         */
        int socket_read_ {0};
        int socket_write_ {0};
        bool is_comms_connected_ {false};
        bool is_motor_engaged_ {false};

        /**
         * @brief For multithreading members
         * 
         */
        std::thread can_read_thread_;
        std::atomic<bool> is_can_reading_ {false};
        std::mutex mtx_pos_;
        std::mutex mtx_vel_;
        
    private:
        /**
         * @brief Convert encoder values to actual joint positions
         * 
         * @param encoder 
         * @return std::array<float, 6> 
         */
        std::array<float, 6> convert_encoder_to_joint(std::array<float, 6> encoder);

        /**
         * @brief Convert encoder value to actual joint position
         * 
         * @param enc 
         * @param idx 
         * @return float 
         */
        float convert_encoder_to_joint(float encoder, int idx);

        /**
         * @brief Convert actual joint positions to encoder values
         * 
         * @param encoder 
         * @return std::array<float, 6> 
         */
        std::array<float, 6> convert_joint_to_encoder(std::array<float, 6> joint);

        /**
         * @brief Convert actual joint position to encoder value
         * 
         * @param joint 
         * @param idx 
         * @return float 
         */
        float convert_joint_to_encoder(float joint, int idx);

        /**
         * @brief Get the node id object
         * 
         * @param msg_id 
         * @return int 
         */
        int get_node_id(int msg_id);

        /**
         * @brief Get the command id object
         * 
         * @param msg_id 
         * @return int 
         */
        int get_command_id(int msg_id);

        /**
         * @brief CAN bus read task
         * 
         */
        void can_read_task();

        /**
         * @brief Handle a CAN message frame
         * 
         * @param frame 
         */
        void can_handle_message(const struct can_frame& frame);

        /**
         * @brief Handles the encoder estimates command
         * 
         * @param frame 
         */
        void encoder_estimates_callback(const struct can_frame& frame);
    };
};

#endif // DBOT_CAN__DBOT_CAN_HPP_
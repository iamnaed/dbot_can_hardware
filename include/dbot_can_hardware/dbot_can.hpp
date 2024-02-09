#ifndef DBOT_CAN__DBOT_CAN_HPP_
#define DBOT_CAN__DBOT_CAN_HPP_

#include "odrive_can.hpp"

#include <cstring>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <array>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <algorithm>

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
    DbotCan()
    {
        can_name_ = "vcan0";
        joint_can_ids_ = {0x001, 0x002, 0x003, 0x004, 0x005, 0x006};

        // Calculate Inverse Reduction Ratios
        for (size_t i = 0; i < joint_reduction_ratios_.size(); i++)
        {
            joint_reduction_ratios_[i] = 48;
        }

        // Calculate Inverse Reduction Ratios
        for (size_t i = 0; i < joint_reduction_ratios_.size(); i++)
        {
            joint_reduction_ratios_inverse_[i] = 1.0f / joint_reduction_ratios_[i];
        }
    }

    /**
     * @brief Initializes the Can Bus using linux's built in SocketCan
     * 
     */
    bool initialize(const DbotCanConfig& config)
    {
        can_name_ = config.can_name;
        joint_can_ids_ = config.joint_can_ids;
        joint_reduction_ratios_ = config.joint_reduction_ratios;

        // Calculate Inverse Reduction Ratios
        for (size_t i = 0; i < joint_reduction_ratios_.size(); i++)
        {
            joint_reduction_ratios_inverse_[i] = 1.0f / joint_reduction_ratios_[i];
        }
        return true;
    }

    /**
     * @brief Creates a connection to the CAN bus 
     * 
     * @return true if successful, false otherwise
     */
    bool connect()
    {
        // Set Socket
        socket_read_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(socket_read_ < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }

        socket_write_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(socket_write_ < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }

        // Specify can_interface device
        int ret_r;
        struct ifreq ifr_read;
        strcpy(ifr_read.ifr_name, can_name_.c_str());
        ret_r = ioctl(socket_read_, SIOCGIFINDEX, &ifr_read);
        if (ret_r < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }

        int ret_w;
        struct ifreq ifr_write;
        strcpy(ifr_write.ifr_name, can_name_.c_str());
        ret_w = ioctl(socket_write_, SIOCGIFINDEX, &ifr_write);
        if (ret_w < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }

        // Bind the socket to interface
        struct sockaddr_can addr_read;
        addr_read.can_family = AF_CAN;
        addr_read.can_ifindex = ifr_read.ifr_ifindex;
        ret_r = bind(socket_read_, (struct sockaddr *)&addr_read, sizeof(addr_read));
        if (ret_r < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }

        struct sockaddr_can addr_write;
        addr_write.can_family = AF_CAN;
        addr_write.can_ifindex = ifr_write.ifr_ifindex;
        ret_w = bind(socket_write_, (struct sockaddr *)&addr_write, sizeof(addr_write));
        if (ret_w < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            is_comms_connected_ = false;
            return false;
        }
        
        // Filtering rules
        //struct can_filter rfilter[1];
        //rfilter[0].can_id = 0x123;
        //rfilter[0].can_mask = CAN_SFF_MASK;
        //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        setsockopt(socket_write_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

        // Start the Read Thread
        is_can_reading_ = true;
        can_read_thread_ = std::thread{&DbotCan::can_read_task, this};
        is_comms_connected_ = true;
        return true;
    }

    /**
     * @brief Disconnects from the CAN bus
     * 
     * @return true if successful, false otherwise
     */
    bool disconnect()
    {
        // Disable CAN reading and stop the threads
        is_can_reading_ = false;
        is_comms_connected_ = false;
        can_read_thread_.join();

        // Close sockets
        int ret_r = close(socket_read_);
        int ret_w = close(socket_write_);
        return (ret_r >= 0) && (ret_w >= 0);
    }

    /**
     * @brief Engages the motor from idle to closed loop control
     * 
     * @return true if successful, false otherwise
     */
    bool engage_motor()
    {
        is_motor_engaged_ = true;
        return true;
    }

    /**
     * @brief Disengages the motor from closed loop control to idle
     * 
     * @return true if successful, false otherwise
     */
    bool disengage_motor()
    {
        is_motor_engaged_ = false;
        return true;
    }

    /**
     * @brief Get the joint positions
     * 
     * @return std::array<float, 6>
     */
    std::array<float, 6> get_position()
    {
        const std::lock_guard<std::mutex> lock(mtx_pos_);
        return joint_positions_;
    }

    /**
     * @brief Get the joint velocities
     * 
     * @return std::array<float, 6> 
     */
    std::array<float, 6> get_velocity()
    {
        const std::lock_guard<std::mutex> lock(mtx_vel_);
        return joint_velocities_;
    }

    /**
     * @brief Set the joint positions
     * 
     * @param pos 
     * @return true if successful, false otherwise
     */
    bool set_position(const std::array<float, 6>& joints)
    {
        // Check communications
        if(!is_comms_connected_)
            return false;
        
        // Check motor
        if(!is_motor_engaged_)
            return false;

        // Convert values for sending
        auto encs = convert_joint_to_encoder(joints);

        // Frame
        for (size_t i = 0; i < joints.size(); i++)
        {
            // Axis and data
            int axis_id = joint_can_ids_[i];
            float value = encs[i];

            // CAN send
            struct can_frame frame;
            int command_id = odrive_can::Command::SetInputPos;
            uint8_t data[8];
            std::memcpy(data, &value, 4);    
            frame.can_id = (axis_id << 5) | (command_id);
            frame.len = 8;
            std::memcpy(frame.data, data, sizeof(data));

            int nbytes = write(socket_write_, &frame, sizeof(frame)); 
            if(nbytes < 0)
                return false;
        }
        
        return true;
    }
    
    /**
     * @brief Get errors in the odrive controllers
     * 
     * @return int 
     */
    int get_errors()
    {
        return 0x0000;
    }

    /**
     * @brief Clear errors in the odrive controllers
     * 
     * @return true if successful, false otherwise
     */
    bool clear_errors()
    {
        return true;
    }
    
private:
    /**
     * @brief Convert encoder values to actual joint positions
     * 
     * @param encoder 
     * @return std::array<float, 6> 
     */
    std::array<float, 6> convert_encoder_to_joint(std::array<float, 6> encoder)
    {
        std::array<float, 6> joints;
        for (size_t i = 0; i < encoder.size(); i++)
        {
            // Inverse of the Reduction ratio are used
            // because multiplication is faster than division
            // Joints [deg] = Encoder [rev] * 360 [deg / rev] * Inverse Reduction Ratio [no unit] 
            joints[i] = encoder[i] * 360.0f * joint_reduction_ratios_inverse_[i];
        }
        
        return joints;
    }

    /**
     * @brief Convert encoder value to actual joint position
     * 
     * @param enc 
     * @param idx 
     * @return float 
     */
    float convert_encoder_to_joint(float encoder, int idx)
    {
        return encoder * 360.0f * joint_reduction_ratios_inverse_[idx];
    }

    /**
     * @brief Convert actual joint positions to encoder values
     * 
     * @param encoder 
     * @return std::array<float, 6> 
     */
    std::array<float, 6> convert_joint_to_encoder(const std::array<float, 6>& joint)
    {
        std::array<float, 6> encs;
        for (size_t i = 0; i < joint.size(); i++)
        {
            // Inverse of the Reduction ratio are used 
            // because multiplication is faster than division
            // Encoder [rev] = Joints [deg] * 0.00277777777 [rev / deg] * Reduction Ratio [no unit]
            encs[i] = joint[i] * 0.00277777777f * joint_reduction_ratios_[i];
        }
        
        return encs;
    }

    /**
     * @brief Convert actual joint position to encoder value
     * 
     * @param joint 
     * @param idx 
     * @return float 
     */
    float convert_joint_to_encoder(float joint, int idx)
    {
        return joint * 0.00277777777f * joint_reduction_ratios_[idx];
    }

    /**
     * @brief Get the node id object
     * 
     * @param msg_id 
     * @return int 
     */
    int get_node_id(int msg_id)
    {
        return (msg_id >> 5);
    }

    /**
     * @brief Get the command id object
     * 
     * @param msg_id 
     * @return int 
     */
    int get_command_id(int msg_id)
    {
        return (msg_id & 0x01F);
    }

    /**
     * @brief CAN bus read task
     * 
     */
    void can_read_task()
    {
         // Set
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));

        while(is_can_reading_)
        {
            // Read
            // This is a blocking function, it waits for an available CAN frame in the buffer
            int nbytes = read(socket_read_, &frame, sizeof(frame));

            // Guard
            if(nbytes < 0)
                continue;

            // Handle CAN message
            can_handle_message(frame);
        }
    }

    /**
     * @brief Handle a CAN message frame
     * 
     * @param frame 
     */
    void can_handle_message(const struct can_frame& frame)
    {
        // Process ID's
        int msg_id = frame.can_id;
        int node_id = get_node_id(msg_id);
        int command_id = get_command_id(msg_id);

        // Guard
        // Check if 'node_id' is inside 'joint_can_ids_'
        auto target_itr = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id);
        bool is_joint_can_id = target_itr != joint_can_ids_.end();
        if(!is_joint_can_id)
            return;

        // Get joint index
        int joint_idx = std::distance(joint_can_ids_.begin(), target_itr);

        // Node Id is inside the container
        switch (command_id)
        {
            case odrive_can::Command::HeartBeatMessage:
                /* Do nothing for now */
                break;
            case odrive_can::Command::GetEncoderEstimates:
                encoder_estimates_callback(frame, joint_idx);
                break;

            default:
                break;
        }
    }

    /**
     * @brief Handles the encoder estimates command
     * 
     * @param frame 
     */
    void encoder_estimates_callback(const struct can_frame& frame, int joint_idx)
    {
        // Process Data
        // 0  1   2  3        4  5  6  7
        // [] [] [] []   --   [] [] [] []
        //   4 bytes     --     4 bytes
        // encoder pos   --   encoder vel
        float pos_buff;
        float vel_buff;
        std::memcpy(&pos_buff, &frame.data[0], 4);
        std::memcpy(&vel_buff, &frame.data[4], 4);
        float jp = convert_encoder_to_joint(pos_buff, joint_idx);
        float jv = convert_encoder_to_joint(vel_buff, joint_idx);

        // Thread safe set functions
        // Position
        set_joint_position(joint_idx, jp);

        // Velocity
        set_joint_velocity(joint_idx, jv);
    }

    /**
     * @brief Set the joint position object
     * 
     * @param idx 
     * @param value 
     */
    void set_joint_position(int idx, float value)
    {
        const std::lock_guard<std::mutex> lock(mtx_pos_);
        joint_positions_[idx] = value;
    }

    /**
     * @brief Set the joint velocity object
     * 
     * @param idx 
     * @param value 
     */
    void set_joint_velocity(int idx, float value)
    {
        const std::lock_guard<std::mutex> lock(mtx_vel_);
        joint_velocities_[idx] = value;
    }

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
    int socket_read_;
    int socket_write_;
    std::atomic<bool> is_comms_connected_;
    std::atomic<bool> is_motor_engaged_;

    /**
     * @brief For multithreading members
     * 
     */
    std::thread can_read_thread_;
    std::atomic<bool> is_can_reading_;
    std::mutex mtx_pos_;
    std::mutex mtx_vel_;
};

#endif // DBOT_CAN__DBOT_CAN_HPP_
#pragma once

#include <string>
#include <vector>
#include <chrono>
#include "odrive_can.hpp"

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace odrive_can;

/**
 * @brief Construct a new Odrive Can:: Odrive Can object
 * 
 * @param can_name 
 */
OdriveCan::OdriveCan(const std::string &can_name, int axis0_can_id, int axis1_can_id) : 
    can_name_(can_name), 
    axis0_can_id_(axis0_can_id),
    axis1_can_id_(axis1_can_id)
{
}

/**
 * @brief Initializes the Odrive Can Bus using linux's built in SocketCan
 * 
 */
bool OdriveCan::initialize()
{
    return true;
}

/**
 * @brief Creates a connection to the CAN bus 
 * 
 * @return true if successfull, false otherwise
 */
bool OdriveCan::connect()
{
    // Set Socket
    socket_read_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_read_ < 0)
    {
        socket_read_ = 0;
        socket_write_ = 0;
        is_comms_active_ = false;
        return false;
    }

    socket_write_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_write_ < 0)
    {
        socket_read_ = 0;
        socket_write_ = 0;
        is_comms_active_ = false;
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
        is_comms_active_ = false;
        return false;
    }

    int ret_w;
    struct ifreq ifr_write;
    strcpy(ifr_write.ifr_name, can_name_.c_str());
    ret_w = ioctl(socket_write_, SIOCGIFINDEX, &ifr_write);
    if (ret_w < 0) {
        socket_read_ = 0;
        socket_write_ = 0;
        is_comms_active_ = false;
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
        is_comms_active_ = false;
        return false;
    }

    struct sockaddr_can addr_write;
    addr_write.can_family = AF_CAN;
    addr_write.can_ifindex = ifr_write.ifr_ifindex;
    ret_w = bind(socket_write_, (struct sockaddr *)&addr_write, sizeof(addr_write));
    if (ret_w < 0) {
        socket_read_ = 0;
        socket_write_ = 0;
        is_comms_active_ = false;
        return false;
    }
    
    // Filtering rules
    //struct can_filter rfilter[1];
    //rfilter[0].can_id = 0x123;
    //rfilter[0].can_mask = CAN_SFF_MASK;
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    setsockopt(socket_write_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    // Start the Read Thread
    is_comms_reading.store(true);
    encoder_read_thread_ = std::thread{encoder_read_task};

    is_comms_active_ = true;
    return true;
}

/**
 * @brief Disconnects from the CAN bus
 * 
 * @return true Success
 * @return false Failed
 */
bool OdriveCan::disconnect()
{
    is_comms_reading.store(false);
    encoder_read_thread_.join();
    int ret_r = close(socket_read_);
    int ret_w = close(socket_write_);
    return (ret_r >= 0) && (ret_w >= 0);
}

/**
 * @brief 
 * 
 * @return true if successfull, false otherwise
 */
bool OdriveCan::engage_motor()
{
    return false;
}

/**
 * @brief 
 * 
 * @param axis 
 * @return true if successfull, false otherwise
 */
bool OdriveCan::engage_motor(const Axis& axis)
{
    return false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return true if successfull, false otherwise
 */
bool OdriveCan::disengage_motor()
{
    return false;
}

/**
 * @brief 
 * 
 * @param axis 
 * @return true if successfull, false otherwise
 */
bool OdriveCan::disengage_motor(const Axis& axis)
{
    return false;
}

/**
 * @brief 
 * 
 * @param axis 
 * @return float 
 */
float OdriveCan::get_position(const Axis& axis)
{
    return 0.0f;
}

/**
 * @brief 
 * 
 * @return std::vector<float> 
 */
std::vector<float> OdriveCan::get_position()
{
    return std::vector<float>();
}

/**
 * @brief 
 * 
 * @param axis 
 * @return float 
 */
float OdriveCan::get_velocity(const Axis& axis)
{
    float vel;

    switch (axis)
    {
    case Axis::Zero:
        mtx_.lock();
        vel = axis0_encoder_vel_;
        mtx_.unlock();
        break;
    case Axis::One:
        mtx_.lock();
        vel = axis1_encoder_vel_;
        mtx_.unlock();
        break;
    }
    
    return vel;
}

/**
 * @brief 
 * 
 * @return std::vector<float> 
 */
std::vector<float> OdriveCan::get_velocity()
{
    return std::vector<float>();
}

/**
 * @brief 
 * 
 * @param axis 
 * @param value 
 * @return true if successful, false otherwise 
 */
bool OdriveCan::set_position(const Axis& axis, float value)
{
    // Check communications
    if(!is_comms_active_)
        return false;
    
    // Check motor
    if(!is_motor_engaged_)
        return false;

    // Frame
    struct can_frame frame;
    int axis_id = get_axis_can_id(axis);
    int command_id = static_cast<int>(Command::SetInputPos);
    uint8_t data[8];
    std::memcpy(data, &value, 4);    
    frame.can_id = (axis_id << 5) | (command_id);
    frame.len = 8;
    std::memcpy(frame.data, data, sizeof(data));

    int nbytes = write(socket_write_, &frame, sizeof(frame)); 
    return nbytes >= 0;
}

/**
 * @brief 
 * 
 * @param value0
 * @param value1
 * @return true if successful, false otherwise 
 */
bool OdriveCan::set_position(float value0, float value1)
{
    bool a0 = OdriveCan::set_position(Axis::Zero, value0);
    bool a1 = OdriveCan::set_position(Axis::One, value1);
    return (a0 && a1);
}

/**
 * @brief 
 * 
 * @return float 
 */
float OdriveCan::get_errors()
{
    return 0.0f;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool OdriveCan::clear_errors()
{
    return false;
}

/**
 * @brief Gets the CAN id for the given axis
 * 
 * @param axis Chosen axis
 * @return CAN id of the axis
 */
int OdriveCan::get_axis_can_id(const Axis& axis)
{
    int axis_can_id;
    switch (axis)
    {
    case Axis::Zero:
        axis_can_id = axis0_can_id_;
        break;
    case Axis::One:
        axis_can_id = axis1_can_id_;
        break;
    }
    return axis_can_id;
}

/**
 * Write a loop where the encoder values are read and 
 * stored in the appropriate variables
*/
void odrive_can::OdriveCan::encoder_read_task()
{
    // Set
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    int axis0_id = get_axis_can_id(Axis::Zero);
    int axis1_id = get_axis_can_id(Axis::One);

    while(true)
    {
        // Break Guard
        if(!is_comms_reading.load())
            break;

        // Read
        int nbytes = read(socket_read_, &frame, sizeof(frame));

        // Guard
        if(nbytes < 0)
            continue;

        // Process ID's
        int msg_id = frame.can_id;
        int node_id = get_node_id(msg_id);
        int command_id = get_command_id(msg_id);
        int command_encoder_estimates = static_cast<int>(Command::GetEncoderEstimates);

        // Disregard other CAN frames
        // Only process GetEncoderEstimates CAN frames [0x009]
        if(command_id != command_encoder_estimates)
            continue;

        // Process Data
        // 0  1   2  3        4  5  6  7
        // [] [] [] []   --   [] [] [] []
        //   4 bytes     --     4 bytes
        // encoder pos   --   encoder vel
        float buff;
        float buff2;
        std::memcpy(&buff, &frame.data[0], 4);
        std::memcpy(&buff2, &frame.data[4], 4);
        if(axis0_id == node_id)
        {
            // Thread safety
            mtx_.lock();
            axis0_encoder_pos_ = buff;
            axis0_encoder_vel_ = buff2;
            mtx_.unlock();
        }
        else if(axis1_id == node_id)
        {
            // Thread safety
            mtx_.lock();
            axis1_encoder_pos_ = buff;
            axis1_encoder_vel_ = buff2;
            mtx_.unlock();
        }
    }
}

/**
 * 
*/
int odrive_can::OdriveCan::get_node_id(int msg_id)
{
    return (msg_id >> 5);
}

/**
 * 
*/
int odrive_can::OdriveCan::get_command_id(int msg_id)
{
    return (msg_id & 0x01F);
}

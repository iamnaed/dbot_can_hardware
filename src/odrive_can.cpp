#pragma once

#include <string>
#include <vector>
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
 * @return true Success
 * @return false Failed
 */
bool OdriveCan::connect()
{
    // Set Socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_ < 0)
    {
        socket_ = 0;
        return false;
    }

    // Specify can_interface device
    int ret;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_name_.c_str());
    ret = ioctl(socket_, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        socket_ = 0;
        return false;
    }

    // Bind the socket to interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        socket_ = 0;
        return false;
    }
    
    // Filtering rules
    //struct can_filter rfilter[1];
    //rfilter[0].can_id = 0x123;
    //rfilter[0].can_mask = CAN_SFF_MASK;
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

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
    int ret = close(socket_);
    return ret >= 0;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool OdriveCan::engage_motor()
{
    return false;
}

/**
 * @brief 
 * 
 * @param axis 
 * @return true 
 * @return false 
 */
bool OdriveCan::engage_motor(const Axis& axis)
{
    return false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool OdriveCan::disengage_motor()
{
    return false;
}

/**
 * @brief 
 * 
 * @param axis 
 * @return true 
 * @return false 
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
    return 0.0f;
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
    if(!is_comms_active)
        return false;
    
    // Check motor
    if(!is_motor_engaged)
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

    int nbytes = write(socket_, &frame, sizeof(frame)); 
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

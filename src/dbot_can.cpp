#include "dbot_can.hpp"

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <algorithm>

using namespace dbot_can;

/**
 * @brief Construct a new dbot can::DbotCan::DbotCan object
 * 
 */
DbotCan::DbotCan()
{
    can_name_ = "vcan0";
    joint_can_ids_[0] = 0x001;
    joint_can_ids_[1] = 0x002;
    joint_can_ids_[2] = 0x003;
    joint_can_ids_[3] = 0x004;
    joint_can_ids_[4] = 0x005;
    joint_can_ids_[5] = 0x006;

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
 * @brief Construct a new dbot can::DbotCan::DbotCan object
 * 
 * @param config 
 */
DbotCan::DbotCan(const DbotCanConfig config)
{
    can_name_ = "vcan0";
    joint_can_ids_ = config.joint_can_ids;
    joint_reduction_ratios_ = config.joint_reduction_ratios;

    // Calculate Inverse Reduction Ratios
    for (size_t i = 0; i < joint_reduction_ratios_.size(); i++)
    {
        joint_reduction_ratios_inverse_[i] = 1.0f / joint_reduction_ratios_[i];
    }
}

/**
 * @brief 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::initialize()
{
    return true;
}

/**
 * @brief Creates a connection to the CAN bus 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::connect()
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
    is_can_reading_.store(true);
    can_read_thread_ = std::thread{can_read_task};

    is_comms_connected_ = true;
    return true;
}

/**
 * @brief 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::disconnect()
{
    // Disable CAN reading and stop the threads
    is_can_reading_.store(false);
    is_comms_connected_ = false;
    can_read_thread_.join();

    // Close sockets
    int ret_r = close(socket_read_);
    int ret_w = close(socket_write_);
    return (ret_r >= 0) && (ret_w >= 0);
}

/**
 * @brief 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::engage_motor()
{
    is_motor_engaged_ = true;
    return true;
}

/**
 * @brief 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::disengage_motor()
{
    is_motor_engaged_ = false;
    return true;
}

/**
 * @brief 
 * 
 * @return std::array<float, 6>
 */
std::array<float, 6> DbotCan::get_position()
{
    float j0, j1, j2, j3, j4, j5;

    mtx_pos_.lock();
    j0 = joint_angles_[0];
    j1 = joint_angles_[1];
    j2 = joint_angles_[2];
    j3 = joint_angles_[3];
    j4 = joint_angles_[4];
    j5 = joint_angles_[5];
    mtx_pos_.unlock();

    return std::array<float, 6>{j0, j0, j0, j0, j0, j0};
}

/**
 * @brief 
 * 
 * @return std::array<float, 6> 
 */
std::array<float, 6> DbotCan::get_velocity()
{
    float j0, j1, j2, j3, j4, j5;

    mtx_vel_.lock();
    j0 = joint_velocities_[0];
    j1 = joint_velocities_[1];
    j2 = joint_velocities_[2];
    j3 = joint_velocities_[3];
    j4 = joint_velocities_[4];
    j5 = joint_velocities_[5];
    mtx_vel_.unlock();

    return std::array<float, 6>{j0, j0, j0, j0, j0, j0};
}

/**
 * @brief 
 * 
 * @param joints 
 * @return true if successful, false otherwise
 */
bool DbotCan::set_position(std::array<float, 6> joints)
{
    // Check communications
    if(!is_comms_connected_)
        return false;
    
    // Check motor
    if(!is_motor_engaged_)
        return false;

    // Convert values for sending
    auto encs = convert_joints_to_encoders(joints);

    // Frame
    for (int i = 0; i < joints.size(); i++)
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
 * @brief 
 * 
 * @return int 
 */
int DbotCan::get_errors()
{
    return 0x0000;
}

/**
 * @brief 
 * 
 * @return true if successful, false otherwise
 */
bool DbotCan::clear_errors()
{
    return true;
}

/**
 * @brief 
 * 
 * @param encoder 
 * @return std::array<float, 6> 
 */
std::array<float, 6> DbotCan::convert_encoders_to_joints(std::array<float, 6> encoder)
{
    std::array<float, 6> joints;
    for (int i = 0; i < encoder.size(); i++)
    {
        // Inverse of the Reduction ratio are used
        // because multiplication is faster than division
        // Joints [deg] = Encoder [rev] * 360 [deg / rev] * Inverse Reduction Ratio [no unit] 
        joints[i] = encoder[i] * 360.0f * joint_reduction_ratios_inverse_[i];
    }
    
    return joints;
}

/**
 * @brief 
 * 
 * @param encoder 
 * @return std::array<float, 6> 
 */
std::array<float, 6> DbotCan::convert_joints_to_encoders(std::array<float, 6> joints)
{
    std::array<float, 6> encs;
    for (int i = 0; i < joints.size(); i++)
    {
        // Inverse of the Reduction ratio are used 
        // because multiplication is faster than division
        // Encoder [rev] = Joints [deg] * 0.00277777777 [rev / deg] * Reduction Ratio [no unit]
        encs[i] = joints[i] * 0.00277777777f * joint_reduction_ratios_[i] ;
    }
    
    return encs;
}

/**
 * @brief Get the node id object
 * 
 * @param msg_id 
 * @return int 
 */
int DbotCan::get_node_id(int msg_id)
{
    return (msg_id >> 5);
}

/**
 * @brief Get the command id object
 * 
 * @param msg_id 
 * @return int 
 */
int DbotCan::get_command_id(int msg_id)
{
    return (msg_id & 0x01F);
}

/**
 * @brief 
 * 
 */
void DbotCan::can_read_task()
{
    // Set
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));

    while(true)
    {
        // Break Guard
        if(!is_can_reading_.load())
            break;

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
 * @brief 
 * 
 * @param frame 
 */
void DbotCan::can_handle_message(const struct can_frame& frame)
{
    // Process ID's
    int msg_id = frame.can_id;
    int node_id = get_node_id(msg_id);
    int command_id = get_command_id(msg_id);

    // Guard
    // Check if 'node_id' is inside 'joint_can_ids_'
    bool is_joint_can_id = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id) != joint_can_ids_.end();
    if(!is_joint_can_id)
        return;

    // Node Id is inside the container
    switch (command_id)
    {
        case odrive_can::Command::HeartBeatMessage:
            /* Do nothing for now */
            break;
        case odrive_can::Command::GetEncoderEstimates:
            encoder_estimates_task(frame);
            break;

        default:
            break;
    }
}

/**
 * @brief 
 * 
 * @param frame 
 */
void DbotCan::encoder_estimates_task(const struct can_frame& frame)
{
    // ID's
    int msg_id = frame.can_id;
    int node_id = get_node_id(msg_id);

    // Get index
    auto target_itr = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id);
    int joint_idx = std::distance(joint_can_ids_.begin(), target_itr);
    
    // Process Data
    // 0  1   2  3        4  5  6  7
    // [] [] [] []   --   [] [] [] []
    //   4 bytes     --     4 bytes
    // encoder pos   --   encoder vel
    float buff;
    float buff2;
    std::memcpy(&buff, &frame.data[0], 4);
    std::memcpy(&buff2, &frame.data[4], 4);

    // Thread safety
    // Position
    mtx_pos_.lock();
    joint_angles_[joint_idx] = buff;
    mtx_pos_.unlock();

    // Velocity
    mtx_vel_.lock();
    joint_velocities_[joint_idx] = buff2;
    mtx_vel_.unlock();
}
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

namespace odrive_can
{
    enum class Axis
    {
        Zero = 0,
        One = 1
    };

    class Command
    {
    public:
        static const int CANOpenNMT = 0x000;
        static const int HeartBeatMessage = 0x001;
        static const int EStop = 0x002;
        static const int GetMotorError = 0x003;
        static const int GetEncoderError = 0x004;
        static const int GetSensorlessError = 0x005;
        static const int SetAxisNodeID = 0x006;
        static const int SetAxisRequestedState = 0x007;
        static const int SetAxisStartupConfig = 0x008;
        static const int GetEncoderEstimates = 0x009;
        static const int GetEncoderCount = 0x00A;
        static const int SetControllerModes = 0x00B;
        static const int SetInputPos = 0x00C;
        static const int SetInputVel = 0x00D;
        static const int SetInputTorque = 0x00E;
        static const int SetLimits = 0x00F;
        static const int StartAnticogging = 0x010;
        static const int SetTrajVelLimit = 0x011;
        static const int SetTrajAccelLimits = 0x012;
        static const int SetTrajInertia = 0x013;
        static const int GetIQ = 0x014;
        static const int GetSensorlessEstimates = 0x015;
        static const int RebootOdrive = 0x016;
        static const int GetBusVoltageAndCurrent = 0x017;
        static const int ClearErrors = 0x018;
        static const int SetLinearCount = 0x019;
        static const int SetPositionGain = 0x01A;
        static const int SetVelGains = 0x01B;
        static const int GetADCVoltage = 0x01C;
        static const int GetControllerError = 0x01D;
        static const int CANOpenHeartbeatMessage = 0x700;
    };

    class OdriveCan
    {
    public:
        /**
         * @brief Construct a new Odrive Can object
         * 
         * @param odrv 
         */
        OdriveCan(const OdriveCan& odrv);

        /**
         * @brief Construct a new Odrive Can object
         * 
         * @param can_name 
         * @param axis0_can_id 
         * @param axis1_can_id 
         */
        OdriveCan(const std::string& can_name, int axis0_can_id, int axis1_can_id);

        /**
         * @brief Initializes the Odrive Can Bus using linux's built in SocketCan
         * 
         * @return true 
         * @return false 
         */
        bool initialize();

        /**
         * @brief Connect to the CAN Bus
         * 
         * @return true 
         * @return false 
         */
        bool connect();

        /**
         * @brief Disconnects from the CAN bus
         * 
         * @return true 
         * @return false 
         */
        bool disconnect();

        /**
         * @brief Activate the motors in both axis
         * 
         * @return true 
         * @return false 
         */
        bool engage_motor();

        /**
         * @brief Activate the motor of the given axis
         * 
         * @param axis 
         * @return true 
         * @return false 
         */
        bool engage_motor(const Axis& axis);

        /**
         * @brief Deactivate the motors in both axis
         * 
         * @return true 
         * @return false 
         */
        bool disengage_motor();

        /**
         * @brief Deactivate the motor of the given axis
         * 
         * @param axis 
         * @return true 
         * @return false 
         */
        bool disengage_motor(const Axis& axis);

        /**
         * @brief Get the position object
         * 
         * @param axis 
         * @return float 
         */
        float get_position(const Axis& axis);

        /**
         * @brief Get the position of both axis
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_position();

        /**
         * @brief Get the velocity of the given axis
         * 
         * @param axis 
         * @return float 
         */
        float get_velocity(const Axis& axis);

        /**
         * @brief Get the velocity of both axis
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_velocity();
        
        /**
         * @brief Set the position of the given axis
         * 
         * @param axis 
         * @param value 
         * @return true 
         * @return false 
         */
        bool set_position(const Axis& axis, float value);

        /**
         * @brief Set the position of the axes
         * 
         * @param value0 
         * @param value1 
         * @return true 
         * @return false 
         */
        bool set_position(float value0, float value1);

        /**
         * @brief Get errors in Odrive
         * 
         * @return float 
         */
        float get_errors();

        /**
         * @brief Clear errors in Odrive
         * 
         * @return true 
         * @return false 
         */
        bool clear_errors();

    private:
        std::string can_name_;
        int axis0_can_id_;
        int axis1_can_id_;
        int socket_read_;
        int socket_write_;
        bool is_comms_active_{false}; 
        bool is_motor_engaged_{false};

        std::atomic<bool> is_comms_reading;
        std::mutex mtx_;
        std::thread encoder_read_thread_;
        float axis0_encoder_pos_;
        float axis0_encoder_vel_;
        float axis1_encoder_pos_;
        float axis1_encoder_vel_;

    private:
        int get_axis_can_id(const Axis& axis);
        int get_node_id(int msg_id);
        int get_command_id(int msg_id);
        void can_read_task();
        void can_handle_message(const struct can_frame& frame);
        void encoder_estimates_task(const struct can_frame& frame);
    };
};
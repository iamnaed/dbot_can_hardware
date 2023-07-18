#pragma once

#include <string>
#include <vector>

namespace odrive_can
{
    enum class Axis
    {
        Zero = 0,
        One = 1
    };

    enum class Command
    {
        CANOpenNMT = 0x000,
        HeartBeatMessage = 0x001,
        EStop = 0x002,
        GetMotorError = 0x003,
        GetEncoderError = 0x004,
        GetSensorlessError = 0x005,
        SetAxisNodeID = 0x006,
        SetAxisRequestedState = 0x007,
        SetAxisStartupConfig = 0x008,
        GetEncoderEstimates = 0x009,
        GetEncoderCount = 0x00A,
        SetControllerModes = 0x00B,
        SetInputPos = 0x00C,
        SetInputVel = 0x00D,
        SetInputTorque = 0x00E,
        SetLimits = 0x00F,
        StartAnticogging = 0x010,
        SetTrajVelLimit = 0x011,
        SetTrajAccelLimits = 0x012,
        SetTrajInertia = 0x013,
        GetIQ = 0x014,
        GetSensorlessEstimates = 0x015,
        RebootOdrive = 0x016,
        GetBusVoltageAndCurrent = 0x017,
        ClearErrors = 0x018,
        SetLinearCount = 0x019,
        SetPositionGain = 0x01A,
        SetVelGains = 0x01B,
        GetADCVoltage = 0x01C,
        GetControllerError = 0x01D,
        CANOpenHeartbeatMessage = 0x700
    };

    class OdriveCan
    {
    public:
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
        int socket_;
        bool is_comms_active{false}; 
        bool is_motor_engaged{false};

    private:
        int get_axis_can_id(const Axis& axis);
    };
};
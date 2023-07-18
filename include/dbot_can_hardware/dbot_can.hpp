#pragma once

#include <cstring>
#include <string>
#include <vector>
#include <memory>

#include "odrive_can.hpp"

namespace dbot_can
{
    enum class Joint
    {
        J0 = 0,
        J1 = 1,
        J2 = 2,
        J3 = 3,
        J4 = 4,
        J5 = 5
    };

    class DbotCan
    {
    public:
        // Construct
        DbotCan();
        DbotCan(const odrive_can::OdriveCan& odrv0, const odrive_can::OdriveCan& odrv1, const odrive_can::OdriveCan& odrv2);

        // Initialization
        void initialize();

        // Pos
        float get_position(const Joint& joint);
        std::vector<float> get_position();

        // Vel
        float get_velocity(const Joint& joint);
        std::vector<float> get_velocity();

        // Error
        float get_errors();
        
        // Command
        bool set_position(const Joint& axis, float value);
        bool set_position(float value0, float value1);
        
        // Communication
        bool connect();
        bool disconnect();

        // Actuator
        bool engage_motor();
        bool disengage_motor();

        // Cleanup
        bool clear_errors();

    private:
        odrive_can::OdriveCan odrv0_;
        odrive_can::OdriveCan odrv1_;
        odrive_can::OdriveCan odrv2_;
    };
};
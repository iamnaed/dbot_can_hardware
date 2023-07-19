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
        /**
         * @brief Construct a new Dbot Can object
         * 
         */
        DbotCan();

        /**
         * @brief Construct a new Dbot Can object
         * 
         * @param odrv0 
         * @param odrv1 
         * @param odrv2 
         */
        DbotCan(const odrive_can::OdriveCan& odrv0, const odrive_can::OdriveCan& odrv1, const odrive_can::OdriveCan& odrv2);

        /**
         * @brief 
         * 
         */
        bool initialize();

        /**
         * @brief Get the position object
         * 
         * @param joint 
         * @return float 
         */
        float get_position(const Joint& joint);

        /**
         * @brief Get the position object
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_position();

        /**
         * @brief Get the velocity object
         * 
         * @param joint 
         * @return float 
         */
        float get_velocity(const Joint& joint);

        /**
         * @brief Get the velocity object
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_velocity();

        /**
         * @brief Get the errors object
         * 
         * @return float 
         */
        float get_errors();
        
        /**
         * @brief Set the position object
         * 
         * @param axis 
         * @param value 
         * @return true if successfull, false otherwise
         */
        bool set_position(const Joint& axis, float value);

        /**
         * @brief Set the position object
         * 
         * @param value0 
         * @param value1 
         * @return true if successfull, false otherwise
         */
        bool set_position(float value0, float value1);
        
        /**
         * @brief 
         * 
         * @return true if successfull, false otherwise
         */
        bool connect();

        /**
         * @brief 
         * 
         * @return true if successfull, false otherwise
         */
        bool disconnect();

        /**
         * @brief 
         * 
         * @return true if successfull, false otherwise
         */
        bool engage_motor();

        /**
         * @brief 
         * 
         * @return true if successfull, false otherwise
         */
        bool disengage_motor();

        /**
         * @brief 
         * 
         * @return true if successfull, false otherwise
         */
        bool clear_errors();

    private:
        odrive_can::OdriveCan odrv0_;
        odrive_can::OdriveCan odrv1_;
        odrive_can::OdriveCan odrv2_;
    };
};
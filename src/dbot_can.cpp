#include "dbot_can.hpp"

dbot_can::DbotCan::DbotCan() : 
    odrv0_{"vcan0", 0x00, 0x01}, 
    odrv1_{"vcan0", 0x02, 0x03}, 
    odrv2_{"vcan0", 0x04, 0x05}
{
}

dbot_can::DbotCan::DbotCan(const odrive_can::OdriveCan &odrv0, const odrive_can::OdriveCan &odrv1, const odrive_can::OdriveCan &odrv2) :
    odrv0_(odrv0), 
    odrv1_(odrv1), 
    odrv2_(odrv2)
{
}
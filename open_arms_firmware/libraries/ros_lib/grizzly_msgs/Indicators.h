#ifndef _ROS_grizzly_msgs_Indicators_h
#define _ROS_grizzly_msgs_Indicators_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace grizzly_msgs
{

  class Indicators : public ros::Msg
  {
    public:
      typedef uint8_t _position_light_type;
      _position_light_type position_light;
      typedef uint8_t _autopilot_light_type;
      _autopilot_light_type autopilot_light;
      typedef uint8_t _battery_light_type;
      _battery_light_type battery_light;
      enum { INDICATOR_ON =  255 };
      enum { INDICATOR_FLASH =  15 };
      enum { INDICATOR_OFF =  0 };

    Indicators():
      position_light(0),
      autopilot_light(0),
      battery_light(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->position_light >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_light);
      *(outbuffer + offset + 0) = (this->autopilot_light >> (8 * 0)) & 0xFF;
      offset += sizeof(this->autopilot_light);
      *(outbuffer + offset + 0) = (this->battery_light >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_light);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->position_light =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_light);
      this->autopilot_light =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->autopilot_light);
      this->battery_light =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_light);
     return offset;
    }

    const char * getType(){ return "grizzly_msgs/Indicators"; };
    const char * getMD5(){ return "96722a571f0c36ed6a0c5468800be83f"; };

  };

}
#endif
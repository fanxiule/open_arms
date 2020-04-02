#ifndef _ROS_grizzly_msgs_Ambience_h
#define _ROS_grizzly_msgs_Ambience_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace grizzly_msgs
{

  class Ambience : public ros::Msg
  {
    public:
      uint8_t body_lights[4];
      typedef uint8_t _beacon_type;
      _beacon_type beacon;
      typedef uint8_t _beep_type;
      _beep_type beep;
      enum { LIGHTS_FRONT_LEFT = 0    };
      enum { LIGHTS_FRONT_RIGHT = 1   };
      enum { LIGHTS_REAR_LEFT = 2     };
      enum { LIGHTS_REAR_RIGHT = 3    };

    Ambience():
      body_lights(),
      beacon(0),
      beep(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->body_lights[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->body_lights[i]);
      }
      *(outbuffer + offset + 0) = (this->beacon >> (8 * 0)) & 0xFF;
      offset += sizeof(this->beacon);
      *(outbuffer + offset + 0) = (this->beep >> (8 * 0)) & 0xFF;
      offset += sizeof(this->beep);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      this->body_lights[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->body_lights[i]);
      }
      this->beacon =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->beacon);
      this->beep =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->beep);
     return offset;
    }

    const char * getType(){ return "grizzly_msgs/Ambience"; };
    const char * getMD5(){ return "8271ef649dd1720612cc15f8990fdf6f"; };

  };

}
#endif
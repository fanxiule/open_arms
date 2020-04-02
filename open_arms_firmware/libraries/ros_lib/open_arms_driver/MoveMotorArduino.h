#ifndef _ROS_SERVICE_MoveMotorArduino_h
#define _ROS_SERVICE_MoveMotorArduino_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_arms_driver
{

static const char MOVEMOTORARDUINO[] = "open_arms_driver/MoveMotorArduino";

  class MoveMotorArduinoRequest : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef bool _direction_type;
      _direction_type direction;
      typedef float _revolution_type;
      _revolution_type revolution;

    MoveMotorArduinoRequest():
      joint_name(""),
      direction(0),
      revolution(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      union {
        bool real;
        uint8_t base;
      } u_direction;
      u_direction.real = this->direction;
      *(outbuffer + offset + 0) = (u_direction.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction);
      offset += serializeAvrFloat64(outbuffer + offset, this->revolution);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
      union {
        bool real;
        uint8_t base;
      } u_direction;
      u_direction.base = 0;
      u_direction.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->direction = u_direction.real;
      offset += sizeof(this->direction);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->revolution));
     return offset;
    }

    const char * getType(){ return MOVEMOTORARDUINO; };
    const char * getMD5(){ return "42a587bbf0e7272a18fbf7cdfca38dd1"; };

  };

  class MoveMotorArduinoResponse : public ros::Msg
  {
    public:
      typedef const char* _response_type;
      _response_type response;

    MoveMotorArduinoResponse():
      response("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_response = strlen(this->response);
      varToArr(outbuffer + offset, length_response);
      offset += 4;
      memcpy(outbuffer + offset, this->response, length_response);
      offset += length_response;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_response;
      arrToVar(length_response, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_response-1]=0;
      this->response = (char *)(inbuffer + offset-1);
      offset += length_response;
     return offset;
    }

    const char * getType(){ return MOVEMOTORARDUINO; };
    const char * getMD5(){ return "6de314e2dc76fbff2b6244a6ad70b68d"; };

  };

  class MoveMotorArduino {
    public:
    typedef MoveMotorArduinoRequest Request;
    typedef MoveMotorArduinoResponse Response;
  };

}
#endif

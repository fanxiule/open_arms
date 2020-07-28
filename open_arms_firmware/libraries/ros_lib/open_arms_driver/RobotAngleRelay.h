#ifndef _ROS_SERVICE_RobotAngleRelay_h
#define _ROS_SERVICE_RobotAngleRelay_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_arms_driver
{

static const char ROBOTANGLERELAY[] = "open_arms_driver/RobotAngleRelay";

  class RobotAngleRelayRequest : public ros::Msg
  {
    public:
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t angle_length;
      typedef float _angle_type;
      _angle_type st_angle;
      _angle_type * angle;

    RobotAngleRelayRequest():
      name_length(0), name(NULL),
      angle_length(0), angle(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->angle_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angle_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angle_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angle_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_length);
      for( uint32_t i = 0; i < angle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t angle_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angle_length);
      if(angle_lengthT > angle_length)
        this->angle = (float*)realloc(this->angle, angle_lengthT * sizeof(float));
      angle_length = angle_lengthT;
      for( uint32_t i = 0; i < angle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_angle;
      u_st_angle.base = 0;
      u_st_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_angle = u_st_angle.real;
      offset += sizeof(this->st_angle);
        memcpy( &(this->angle[i]), &(this->st_angle), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return ROBOTANGLERELAY; };
    const char * getMD5(){ return "09bd1c07f75cbd4c74335213c43c21ef"; };

  };

  class RobotAngleRelayResponse : public ros::Msg
  {
    public:
      typedef bool _motion_done_type;
      _motion_done_type motion_done;
      typedef bool _overtorque_type;
      _overtorque_type overtorque;

    RobotAngleRelayResponse():
      motion_done(0),
      overtorque(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motion_done;
      u_motion_done.real = this->motion_done;
      *(outbuffer + offset + 0) = (u_motion_done.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motion_done);
      union {
        bool real;
        uint8_t base;
      } u_overtorque;
      u_overtorque.real = this->overtorque;
      *(outbuffer + offset + 0) = (u_overtorque.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->overtorque);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motion_done;
      u_motion_done.base = 0;
      u_motion_done.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motion_done = u_motion_done.real;
      offset += sizeof(this->motion_done);
      union {
        bool real;
        uint8_t base;
      } u_overtorque;
      u_overtorque.base = 0;
      u_overtorque.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->overtorque = u_overtorque.real;
      offset += sizeof(this->overtorque);
     return offset;
    }

    const char * getType(){ return ROBOTANGLERELAY; };
    const char * getMD5(){ return "f7780197b9a6bf5e785c44c681ea2a03"; };

  };

  class RobotAngleRelay {
    public:
    typedef RobotAngleRelayRequest Request;
    typedef RobotAngleRelayResponse Response;
  };

}
#endif

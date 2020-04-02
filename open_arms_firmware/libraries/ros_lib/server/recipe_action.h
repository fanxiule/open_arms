#ifndef _ROS_SERVICE_recipe_action_h
#define _ROS_SERVICE_recipe_action_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace server
{

static const char RECIPE_ACTION[] = "server/recipe_action";

  class recipe_actionRequest : public ros::Msg
  {
    public:
      typedef const char* _pasta_type;
      _pasta_type pasta;
      typedef const char* _sauce_type;
      _sauce_type sauce;

    recipe_actionRequest():
      pasta(""),
      sauce("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_pasta = strlen(this->pasta);
      varToArr(outbuffer + offset, length_pasta);
      offset += 4;
      memcpy(outbuffer + offset, this->pasta, length_pasta);
      offset += length_pasta;
      uint32_t length_sauce = strlen(this->sauce);
      varToArr(outbuffer + offset, length_sauce);
      offset += 4;
      memcpy(outbuffer + offset, this->sauce, length_sauce);
      offset += length_sauce;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_pasta;
      arrToVar(length_pasta, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pasta; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pasta-1]=0;
      this->pasta = (char *)(inbuffer + offset-1);
      offset += length_pasta;
      uint32_t length_sauce;
      arrToVar(length_sauce, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sauce; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sauce-1]=0;
      this->sauce = (char *)(inbuffer + offset-1);
      offset += length_sauce;
     return offset;
    }

    const char * getType(){ return RECIPE_ACTION; };
    const char * getMD5(){ return "74e89521d796f1287d083bbcc7f75682"; };

  };

  class recipe_actionResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    recipe_actionResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return RECIPE_ACTION; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class recipe_action {
    public:
    typedef recipe_actionRequest Request;
    typedef recipe_actionResponse Response;
  };

}
#endif

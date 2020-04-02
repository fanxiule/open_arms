#ifndef _ROS_SERVICE_DegToRev_h
#define _ROS_SERVICE_DegToRev_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_arms_driver
{

static const char DEGTOREV[] = "open_arms_driver/DegToRev";

  class DegToRevRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _deg_type;
      _deg_type deg;

    DegToRevRequest():
      name(""),
      deg(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->deg);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->deg));
     return offset;
    }

    const char * getType(){ return DEGTOREV; };
    const char * getMD5(){ return "12bcd230798035b9644ddf1f2fca87de"; };

  };

  class DegToRevResponse : public ros::Msg
  {
    public:
      typedef const char* _response_type;
      _response_type response;

    DegToRevResponse():
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

    const char * getType(){ return DEGTOREV; };
    const char * getMD5(){ return "6de314e2dc76fbff2b6244a6ad70b68d"; };

  };

  class DegToRev {
    public:
    typedef DegToRevRequest Request;
    typedef DegToRevResponse Response;
  };

}
#endif

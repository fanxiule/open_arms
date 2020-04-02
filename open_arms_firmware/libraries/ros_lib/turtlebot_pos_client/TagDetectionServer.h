#ifndef _ROS_turtlebot_pos_client_TagDetectionServer_h
#define _ROS_turtlebot_pos_client_TagDetectionServer_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace turtlebot_pos_client
{

  class TagDetectionServer : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _tag_id_type;
      _tag_id_type tag_id;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;

    TagDetectionServer():
      header(),
      tag_id(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_tag_id;
      u_tag_id.real = this->tag_id;
      *(outbuffer + offset + 0) = (u_tag_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tag_id);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_tag_id;
      u_tag_id.base = 0;
      u_tag_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tag_id = u_tag_id.real;
      offset += sizeof(this->tag_id);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot_pos_client/TagDetectionServer"; };
    const char * getMD5(){ return "7298744e46e53bcc3ea6b71994331c6d"; };

  };

}
#endif
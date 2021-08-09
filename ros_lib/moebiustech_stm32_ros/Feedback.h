#ifndef _ROS_moebiustech_stm32_ros_Feedback_h
#define _ROS_moebiustech_stm32_ros_Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "moebiustech_stm32_ros/JointFeedback.h"

namespace moebiustech_stm32_ros
{

  class Feedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      moebiustech_stm32_ros::JointFeedback drivers[2];

    Feedback():
      header(),
      drivers()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 2; i++){
      offset += this->drivers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 2; i++){
      offset += this->drivers[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "moebiustech_stm32_ros/Feedback"; };
    virtual const char * getMD5() override { return "23c8e1f51ba6794177a0678435d8b615"; };

  };

}
#endif

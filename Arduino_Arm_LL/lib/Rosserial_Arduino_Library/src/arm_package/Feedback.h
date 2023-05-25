#ifndef _ROS_arm_package_Feedback_h
#define _ROS_arm_package_Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_package
{

  class Feedback : public ros::Msg
  {
    public:
      typedef int16_t _Pot_1_type;
      _Pot_1_type Pot_1;
      typedef int16_t _Pot_2_type;
      _Pot_2_type Pot_2;
      typedef int16_t _Pot_3_type;
      _Pot_3_type Pot_3;
      typedef int16_t _Pot_4_type;
      _Pot_4_type Pot_4;
      typedef int16_t _Pot_5_type;
      _Pot_5_type Pot_5;

    Feedback():
      Pot_1(0),
      Pot_2(0),
      Pot_3(0),
      Pot_4(0),
      Pot_5(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_1;
      u_Pot_1.real = this->Pot_1;
      *(outbuffer + offset + 0) = (u_Pot_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pot_1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Pot_1);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_2;
      u_Pot_2.real = this->Pot_2;
      *(outbuffer + offset + 0) = (u_Pot_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pot_2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Pot_2);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_3;
      u_Pot_3.real = this->Pot_3;
      *(outbuffer + offset + 0) = (u_Pot_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pot_3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Pot_3);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_4;
      u_Pot_4.real = this->Pot_4;
      *(outbuffer + offset + 0) = (u_Pot_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pot_4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Pot_4);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_5;
      u_Pot_5.real = this->Pot_5;
      *(outbuffer + offset + 0) = (u_Pot_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pot_5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Pot_5);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_1;
      u_Pot_1.base = 0;
      u_Pot_1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pot_1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Pot_1 = u_Pot_1.real;
      offset += sizeof(this->Pot_1);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_2;
      u_Pot_2.base = 0;
      u_Pot_2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pot_2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Pot_2 = u_Pot_2.real;
      offset += sizeof(this->Pot_2);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_3;
      u_Pot_3.base = 0;
      u_Pot_3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pot_3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Pot_3 = u_Pot_3.real;
      offset += sizeof(this->Pot_3);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_4;
      u_Pot_4.base = 0;
      u_Pot_4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pot_4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Pot_4 = u_Pot_4.real;
      offset += sizeof(this->Pot_4);
      union {
        int16_t real;
        uint16_t base;
      } u_Pot_5;
      u_Pot_5.base = 0;
      u_Pot_5.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pot_5.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Pot_5 = u_Pot_5.real;
      offset += sizeof(this->Pot_5);
     return offset;
    }

    virtual const char * getType() override { return "arm_package/Feedback"; };
    virtual const char * getMD5() override { return "d4fb9655337ecf1cff314d4a2537c34d"; };

  };

}
#endif

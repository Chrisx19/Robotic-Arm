#ifndef _ROS_arm_package_Joints_h
#define _ROS_arm_package_Joints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_package
{

  class Joints : public ros::Msg
  {
    public:
      typedef bool _EN_type;
      _EN_type EN;
      typedef float _Joint_1_type;
      _Joint_1_type Joint_1;
      typedef float _Joint_2_type;
      _Joint_2_type Joint_2;
      typedef float _Joint_3_type;
      _Joint_3_type Joint_3;
      typedef float _Joint_4_type;
      _Joint_4_type Joint_4;
      typedef float _Joint_5_type;
      _Joint_5_type Joint_5;
      typedef float _Joint_6_type;
      _Joint_6_type Joint_6;

    Joints():
      EN(0),
      Joint_1(0),
      Joint_2(0),
      Joint_3(0),
      Joint_4(0),
      Joint_5(0),
      Joint_6(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_EN;
      u_EN.real = this->EN;
      *(outbuffer + offset + 0) = (u_EN.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->EN);
      union {
        float real;
        uint32_t base;
      } u_Joint_1;
      u_Joint_1.real = this->Joint_1;
      *(outbuffer + offset + 0) = (u_Joint_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_1);
      union {
        float real;
        uint32_t base;
      } u_Joint_2;
      u_Joint_2.real = this->Joint_2;
      *(outbuffer + offset + 0) = (u_Joint_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_2);
      union {
        float real;
        uint32_t base;
      } u_Joint_3;
      u_Joint_3.real = this->Joint_3;
      *(outbuffer + offset + 0) = (u_Joint_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_3);
      union {
        float real;
        uint32_t base;
      } u_Joint_4;
      u_Joint_4.real = this->Joint_4;
      *(outbuffer + offset + 0) = (u_Joint_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_4);
      union {
        float real;
        uint32_t base;
      } u_Joint_5;
      u_Joint_5.real = this->Joint_5;
      *(outbuffer + offset + 0) = (u_Joint_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_5);
      union {
        float real;
        uint32_t base;
      } u_Joint_6;
      u_Joint_6.real = this->Joint_6;
      *(outbuffer + offset + 0) = (u_Joint_6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Joint_6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Joint_6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Joint_6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Joint_6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_EN;
      u_EN.base = 0;
      u_EN.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->EN = u_EN.real;
      offset += sizeof(this->EN);
      union {
        float real;
        uint32_t base;
      } u_Joint_1;
      u_Joint_1.base = 0;
      u_Joint_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_1 = u_Joint_1.real;
      offset += sizeof(this->Joint_1);
      union {
        float real;
        uint32_t base;
      } u_Joint_2;
      u_Joint_2.base = 0;
      u_Joint_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_2 = u_Joint_2.real;
      offset += sizeof(this->Joint_2);
      union {
        float real;
        uint32_t base;
      } u_Joint_3;
      u_Joint_3.base = 0;
      u_Joint_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_3 = u_Joint_3.real;
      offset += sizeof(this->Joint_3);
      union {
        float real;
        uint32_t base;
      } u_Joint_4;
      u_Joint_4.base = 0;
      u_Joint_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_4 = u_Joint_4.real;
      offset += sizeof(this->Joint_4);
      union {
        float real;
        uint32_t base;
      } u_Joint_5;
      u_Joint_5.base = 0;
      u_Joint_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_5 = u_Joint_5.real;
      offset += sizeof(this->Joint_5);
      union {
        float real;
        uint32_t base;
      } u_Joint_6;
      u_Joint_6.base = 0;
      u_Joint_6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Joint_6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Joint_6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Joint_6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Joint_6 = u_Joint_6.real;
      offset += sizeof(this->Joint_6);
     return offset;
    }

    virtual const char * getType() override { return "arm_package/Joints"; };
    virtual const char * getMD5() override { return "c923870f9c9600549593f3f3d28b62a3"; };

  };

}
#endif

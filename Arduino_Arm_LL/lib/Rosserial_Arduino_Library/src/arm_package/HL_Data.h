#ifndef _ROS_arm_package_HL_Data_h
#define _ROS_arm_package_HL_Data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_package
{

  class HL_Data : public ros::Msg
  {
    public:
      typedef bool _EN_type;
      _EN_type EN;
      typedef int16_t _Camera_Distance_type;
      _Camera_Distance_type Camera_Distance;
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
      typedef bool _Gripper_type;
      _Gripper_type Gripper;

    HL_Data():
      EN(0),
      Camera_Distance(0),
      Joint_1(0),
      Joint_2(0),
      Joint_3(0),
      Joint_4(0),
      Joint_5(0),
      Gripper(0)
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
        int16_t real;
        uint16_t base;
      } u_Camera_Distance;
      u_Camera_Distance.real = this->Camera_Distance;
      *(outbuffer + offset + 0) = (u_Camera_Distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Camera_Distance.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Camera_Distance);
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
        bool real;
        uint8_t base;
      } u_Gripper;
      u_Gripper.real = this->Gripper;
      *(outbuffer + offset + 0) = (u_Gripper.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Gripper);
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
        int16_t real;
        uint16_t base;
      } u_Camera_Distance;
      u_Camera_Distance.base = 0;
      u_Camera_Distance.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Camera_Distance.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Camera_Distance = u_Camera_Distance.real;
      offset += sizeof(this->Camera_Distance);
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
        bool real;
        uint8_t base;
      } u_Gripper;
      u_Gripper.base = 0;
      u_Gripper.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Gripper = u_Gripper.real;
      offset += sizeof(this->Gripper);
     return offset;
    }

    virtual const char * getType() override { return "arm_package/HL_Data"; };
    virtual const char * getMD5() override { return "80c6a8affbfdc0c23fb5c1123155541a"; };

  };

}
#endif

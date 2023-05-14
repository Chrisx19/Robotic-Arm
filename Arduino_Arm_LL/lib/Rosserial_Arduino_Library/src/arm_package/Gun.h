#ifndef _ROS_arm_package_Gun_h
#define _ROS_arm_package_Gun_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_package
{

  class Gun : public ros::Msg
  {
    public:
      typedef bool _Gun_mode_type;
      _Gun_mode_type Gun_mode;
      typedef bool _Fire_type;
      _Fire_type Fire;
      typedef bool _Reload_type;
      _Reload_type Reload;
      typedef float _Turn_type;
      _Turn_type Turn;

    Gun():
      Gun_mode(0),
      Fire(0),
      Reload(0),
      Turn(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Gun_mode;
      u_Gun_mode.real = this->Gun_mode;
      *(outbuffer + offset + 0) = (u_Gun_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Gun_mode);
      union {
        bool real;
        uint8_t base;
      } u_Fire;
      u_Fire.real = this->Fire;
      *(outbuffer + offset + 0) = (u_Fire.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Fire);
      union {
        bool real;
        uint8_t base;
      } u_Reload;
      u_Reload.real = this->Reload;
      *(outbuffer + offset + 0) = (u_Reload.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Reload);
      union {
        float real;
        uint32_t base;
      } u_Turn;
      u_Turn.real = this->Turn;
      *(outbuffer + offset + 0) = (u_Turn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Turn.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Turn.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Turn.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Turn);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Gun_mode;
      u_Gun_mode.base = 0;
      u_Gun_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Gun_mode = u_Gun_mode.real;
      offset += sizeof(this->Gun_mode);
      union {
        bool real;
        uint8_t base;
      } u_Fire;
      u_Fire.base = 0;
      u_Fire.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Fire = u_Fire.real;
      offset += sizeof(this->Fire);
      union {
        bool real;
        uint8_t base;
      } u_Reload;
      u_Reload.base = 0;
      u_Reload.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Reload = u_Reload.real;
      offset += sizeof(this->Reload);
      union {
        float real;
        uint32_t base;
      } u_Turn;
      u_Turn.base = 0;
      u_Turn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Turn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Turn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Turn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Turn = u_Turn.real;
      offset += sizeof(this->Turn);
     return offset;
    }

    virtual const char * getType() override { return "arm_package/Gun"; };
    virtual const char * getMD5() override { return "8146b618e51a7e077c442dd185584475"; };

  };

}
#endif

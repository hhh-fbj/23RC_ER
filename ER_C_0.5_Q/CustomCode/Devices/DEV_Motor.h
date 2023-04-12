#ifndef __DEV_MOTOR_H
#define __DEV_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "BSP_CAN.h"
#ifdef __cplusplus
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
template<typename Type> 
void _motor_Constrain(Type *x, Type Min, Type Max) 
{
  if(*x < Min) *x = Min;
  else if(*x > Max) *x = Max;
  else{;}
}
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
/**
 *@brief  Base class for motors.
 */
class MotorBase
{
public:
    MotorBase(uint8_t id) : ID(id) {}
    virtual ~MotorBase() {}
    
    virtual void update(uint8_t can_rx_data[]) = 0;
    virtual bool CheckID(uint32_t StdID) const { return StdID == this->REC_ID_INIT() + (uint32_t)ID; }
    float getAngle() const { return angle; }
	float getencoder() const { return encoder; }
    float get_totalencoder() const {return total_encoder; }
		
    const uint8_t ID = 0;
	
    float encoder_offset = 0;
	float Out = 0; /*!< Output ampere value that sent to ESC */

	uint16_t encoder = 0; 
protected:
    float angle = 0,  last_encoder = 0;
    int32_t total_encoder = 0;
    bool encoder_is_init = false;

    virtual uint32_t REC_ID_INIT() const { return 0; };

    void update_angle(uint8_t can_rx_data[])
    {
      encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
      if(encoder_is_init)
      {
        if (this->encoder - this->last_encoder > 4096)
            this->round_cnt--;
        else if (this->encoder - this->last_encoder < -4096)
            this->round_cnt++;
      }
      else
      {
        encoder_offset = encoder;
        encoder_is_init = true;
      }
      this->last_encoder = this->encoder;
      total_encoder = round_cnt * 8192 + encoder - encoder_offset;
      angle = total_encoder / ENCODER_ANGLE_RATIO();
    }

    void Reset_angle()
    {
        this->last_encoder = this->encoder;
        // this->total_encoder = this->encoder - this->encoder_offset;
        this->round_cnt = 0;
    }

private:
    //uint16_t encoder = 0;  修改为public
    int32_t round_cnt = 0;
    virtual int16_t ENCODER_MAX() const { return 8192; }
    virtual float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }
    virtual float MAX_CURRENT() const { return 65535; }
};


class MotorSpeed : public MotorBase
{
public:
    MotorSpeed(uint8_t id) : MotorBase(id) {}
    virtual ~MotorSpeed(){};
    virtual float getSpeed() const { return this->speed; }
    virtual void update(uint8_t can_rx_data[]) = 0;

protected:
    int16_t speed = 0;
    void update_speed(uint8_t can_rx_data[])
    {
      this->speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};

#if 0
/**
  @brief  Motor that used ESC 820R.
*/
class Motor_820R : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_820R(uint8_t id) : MotorSpeed(id) {}

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
    }
};

/**
  @brief  GM3510.
*/
class Motor_GM3510 : public MotorBase
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0; }
    virtual float MAX_CURRENT() const { return 29000; }
    Motor_GM3510(uint8_t id) : MotorBase(id) {}

    int16_t getTorque() const { return torque; }

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        torque = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }

protected:
    int16_t torque = 0;
};

/**
  @brief  Motor 6623.
*/
class Motor_6623 : public MotorBase
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 5000; }
    void setEncoderOffset(uint16_t offset) { 
      this->encoder_offset = offset;
      this->last_encoder   = offset;
      this->encoder_is_init = true;
    }
    enum
    {
        Yaw = 1,
        Pitch,
        Roll,
        Resv,
        Ex1,
        Ex2,
        Ex3,
        Ex4
    };
    Motor_6623(uint8_t id) : MotorBase(id) {}

    int16_t getTorque() const { return torque; }

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        torque = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }

protected:
    int16_t torque = 0;
};

#endif

/**
  @brief  Motor M2006 that used ESC C610.
*/
class Motor_M2006 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const override{ return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const override{ return 10000; }
    Motor_M2006(uint8_t id) : MotorSpeed(id) {}
    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->torque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
    }
    int16_t getTorque() const { return torque; }

    virtual void Reset()
    {
        this->Reset_angle();
    }

protected:
    int16_t torque = 0;
};

/**
  @brief  Motor M3508 that used ESC C620.
*/
class Motor_M3508 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const override{ return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const override{ return 16000; }
    Motor_M3508(uint8_t id) : MotorSpeed(id) {}
    uint8_t getTempature() const { return temperature; }
    int16_t getGivenCurrent() const { return givenCurrent; }

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        givenCurrent = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        temperature = can_rx_data[6];
    }

protected:
    uint8_t temperature = 0;
    int16_t givenCurrent = 0;
};

/**
  @brief  GM6020.
*/
class Motor_GM6020 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const override{ return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x2ff; }
    virtual float MAX_CURRENT() const override{ return 30000; }
    void setEncoderOffset(uint16_t offset) { 
        this->encoder_offset = offset;
        this->last_encoder   = offset;
        this->encoder_is_init = true;
    }
    Motor_GM6020(uint8_t id) : MotorSpeed(id) {}
    uint8_t getTempature() const { return temperature; }
    int16_t getGivenCurrent() const { return givenCurrent; }
    
    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->givenCurrent = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        this->temperature = can_rx_data[6];
    }

    virtual void Reset()
    {
        this->Reset_angle();
    }

protected:
    uint8_t temperature = 0;
    int16_t givenCurrent = 0;
};

/**
  Send CAN communication object.
*/
template <class MotorType, int N>
void MotorMsgSend(CAN_HandleTypeDef *hcan, MotorType (&motors)[N])
{
    uint8_t TxDataLow[8] = {0};
    uint8_t TxDataHigh[8] = {0};
    int16_t constrain_Out = 0;
    bool low = false;
    bool high = false;
    for (int i = 0; i < N; i++)
    {
        _motor_Constrain(&motors[i].Out, -motors[0].MAX_CURRENT(), motors[0].MAX_CURRENT());
        constrain_Out = motors[i].Out;
        if (motors[i].ID <= 4 && motors[i].ID > 0)
        {
            low = true;
            TxDataLow[motors[i].ID * 2 - 2] = (constrain_Out >> 8) & 0xff;
            TxDataLow[motors[i].ID * 2 - 1] = constrain_Out & 0xff;
        }
        else if (motors[i].ID <= 8 && motors[i].ID > 4)
        {
            high = true;
            TxDataHigh[motors[i].ID * 2 - 10] = (constrain_Out >> 8) & 0xff;
            TxDataHigh[motors[i].ID * 2 - 9] = constrain_Out & 0xff;
        }
    }

    if (low)
        CANx_SendData(hcan, motors[0].SEND_ID_LOW(), TxDataLow, 8);

    if (high)
        CANx_SendData(hcan, motors[0].SEND_ID_HIGH(), TxDataHigh, 8);
}

template <class MotorType>
void MotorMsgSend(CAN_HandleTypeDef *hcan, MotorType &motor)
{
    MotorType motor_arr[1] = {motor};
    MotorMsgSend(hcan, motor_arr);
}
#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

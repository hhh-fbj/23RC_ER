#include "ALGO_PID.h"
/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SystemTick_Fun PIDTimer::Get_SystemTick = NULL;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
template<typename Type> 
Type _pid_Abs(Type x) {return ((x > 0) ? x : -x);}

template<typename Type> 
void _pid_Constrain(Type *x, Type Min, Type Max) 
{
  if(*x < Min) *x = Min;
  else if(*x > Max) *x = Max;
  else{;}
}
/* function prototypes -------------------------------------------------------*/
uint8_t PIDTimer::UpdataTimeStamp(void)
{
  uint32_t now_time;
  
  /*Check `Get_SystemTick` */
  if(PIDTimer::Get_SystemTick != NULL)
  {
    /*Convert to system time*/
    if (last_time == 0)
    {
      last_time = PIDTimer::Get_SystemTick();
      return 1;
    }
    now_time = PIDTimer::Get_SystemTick();

    /*Overflow*/
    if (now_time < last_time)
      dt = (float)(now_time + (0xFFFFFFFF - last_time));
    else
      dt = (float)(now_time - last_time);

    last_time = now_time;

    dt *= 0.000001f;
    
    return 0;
  }
  else{
    dt = 0;
    return 1;
  }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t PIDTimer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
  if(getTick_fun != NULL)
  {
    PIDTimer::Get_SystemTick = getTick_fun;
    return 1;
  }
  else 
    return 0;
}

/* IncrementPID---------------------------------------------------*/
float IncrementPID::Cal()
{
	/*Error time*/
  if (UpdataTimeStamp()){return 0;}
	Error = Target - Current;

  if (_pid_Abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }
	
	/* Using Low-Pass Filter to preprocess*/
  Error = LowPass_error.f(Error);

  P_Term = Kp * (Error - pre_error);

	/* PID with Changing integration rate */
	float I_VarSpeedf = 0;
	
	if (_pid_Abs(Error) <= VarSpeed_I_B)
  {
	  I_VarSpeedf = 1;
  }
	else if (_pid_Abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
  {
	  I_VarSpeedf = (VarSpeed_I_A - (_pid_Abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
  }
  
  if(Ki != 0)
  {
    integral_e = I_VarSpeedf * Error * dt;
    /*Constrain*/
    _pid_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
  }
  else
  {
    integral_e = 0;
  }
  
	
  /* Using Integral separation */
  if (_pid_Abs(Error) < I_SeparThresh)
  { 
    I_Term = Ki * integral_e;
    /*Constarin*/
    _pid_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    I_Term = 0;
  }
  
	
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current);
  else
    d_err = (Error - 2.0f*pre_error + prev_error);

	
  D_Term = Kd * d_err;

  prev_error = pre_error;
  pre_error = Error;

  Out += (P_Term + I_Term + D_Term);
  
  /* Constarin */
  _pid_Constrain(&Out, -Out_Max, Out_Max);
  
  return Out;
}

void IncrementPID::Reset()
{
	Error = 0;
	Current = 0;
	pre_Current = 0;
	P_Term = 0;
	I_Term = 0;
	D_Term = 0;
	Out = 0;
	integral_e = 0;
	prev_error = 0;
	
}



/* PositionPID---------------------------------------------------*/
float PositionPID::Cal()
{
	/*Error time*/
  if (UpdataTimeStamp()){return 0;}
  // CZJ 
  Error = (error_turn?(Current - Target):(Target - Current));

  if (_pid_Abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }
	
	/* Using Low-Pass Filter to preprocess*/
  Error = LowPass_error.f(Error);
	
	//变结构---------------------------------------------------------------STAR
	if(a_p != 0)
	{
		//K_p = a_p + b_p * (1 - 1/exp(c_p*|e(t)|)) 
		Kp=a_p+b_p*(1-1/exp(c_p*abs(Error)));
	}
	if(a_i != 0)
	{
		//K_i = a_i * (1 / exp(c_i*|e(t)|))
		Ki=a_i*(1/exp(c_i*abs(Error)));
	}
	//变结构---------------------------------------------------------------OVER

  P_Term = Kp * Error;

	/* PID with Changing integration rate */
	float I_VarSpeedf = 0;
	if (_pid_Abs(Error) <= VarSpeed_I_B)
	  I_VarSpeedf = 1;
	else if (_pid_Abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
	  I_VarSpeedf = (VarSpeed_I_A - (_pid_Abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
  
  if(Ki != 0){
    integral_e += I_VarSpeedf * Error * dt;
    /*Constrain*/
    _pid_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
  }
  else{
    integral_e = 0;
  }
  
  /* Using Integral separation */
  if (_pid_Abs(Error) < I_SeparThresh)
  {
    // TEXT 外环i分离
    if(YN_I_TOTAL == 1)
    {
      if(((pre_error > 0 && Error > 0) || (pre_error < 0 && Error < 0)) && abs(Error) < I_IN_MATH)
      {
        integral_e *= (abs(Error) / I_IN_MATH);
      }
      // else if((pre_error >0 && Error < 0) || (pre_error <0 && Error > 0))
      // {
      //   integral_e = 0;
      // }
    }
    else if(YN_I_TOTAL == 2)
    {
      if(((pre_error > 0 && Error > 0) || (pre_error < 0 && Error < 0)) && abs(Error) < I_IN_MATH)
      {
        integral_e *= (abs(Error) / I_IN_MATH);
      }
      else if((pre_error >0 && Error < 0) || (pre_error <0 && Error > 0))
      {
        integral_e = 0;
      }
    }
    I_Term = Ki * integral_e;
    /*Constarin*/
    _pid_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    if(I_SeI_C == 0){integral_e = 0;}
    // integral_e = (I_SeI_C?integral_e:0);
    I_Term = 0;
  }
  
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current) / dt;
  else
    d_err = (Error - pre_error) / dt;

	
  D_Term = Kd * d_err;

  pre_error = Error;

  Out = P_Term + I_Term + D_Term;
  
  /* Constarin */
  _pid_Constrain(&Out, -Out_Max, Out_Max);
  
  return Out;
}


void PositionPID::Reset()
{
	Error = 0;
	Current = 0;
	pre_Current = 0;
	P_Term = 0;
	I_Term = 0;
	D_Term = 0;
	Out = 0;
	integral_e = 0;
}



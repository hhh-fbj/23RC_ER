#ifndef __ALGO_FILTER_H
#define __ALGO_FILTER_H

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <algorithm>
#include <string.h>

/* Exported function declarations --------------------------------------------*/
/* LowPassFilter */
class LowPassFilter
{
public:
  /**
    @brief trust (0,1) 
   */
	LowPassFilter(float trust = 1): Trust(trust)
  {
      now_num = last_num = 0;
  } 
  ~LowPassFilter(){};
  float Trust;
  void operator<< (const float& );
  void operator>> (float& );
  float f(float num);
protected:
  void in(float num);
  float out();
private:
  float now_num;
  float last_num;
};

/* MedianFilter	*/
template<int Length> 	
class MedianFilter
{
  /**
    @brief �˲�����(1,100)
   */
  public:
	MedianFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		flag = Length;
		where_num = 0;
	} 						
  	~MedianFilter(){}; 
    void operator >> (float& num){ num = out();}
    void operator << (const float& num){in(num);}
    float f(float num)
    {
      in(num);
      return (out());
    }
  protected:
  	void in(float num)
    {
      now_num = num;
      /* flag=LengthȻ��ݼ���֤�����ڶ�����Ч��ֵ */
      flag > 0? flag-- : 0;										
      buffer_num[where_num++] = num;
      where_num %= Length; 
    }
    
  	float out()
    {
      if(flag>0)
        return now_num;
      else
        {
          /* ׼������ */
          memcpy(sort_num,buffer_num,sizeof(sort_num));	
          std::sort(sort_num,sort_num+Length);
          return sort_num[int(Length/2)];
        }
    }
    
  private:
  	float buffer_num[Length];
  	float sort_num[Length];
	float now_num;
	int flag,where_num;
};

/* MeanFilter */
template<int Length> 	
class MeanFilter
{
  public:
  /**
    @brief �˲�����(1,100)
   */
	MeanFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		for(int x = 0 ; x < Length; x++) buffer_num[x] = 0;
		flag = Length;
		where_num = 0;
		sum = 0;
	} 						
  	~MeanFilter(){}; 
    void operator >> (float& num){ num = out();}	
    void operator << (const float& num){in(num);}
    float f(float num)
    {
      in(num);
      return (out());
    }
  protected:
  	void in(float num)
    {
      now_num = num;
      sum -= buffer_num[where_num];			  /*<! sum��ȥ��ֵ */
      sum += num;													/*<! sum������ֵ */
      buffer_num[where_num++] = num;
      flag > 0? flag-- : 0;								/*<!flag=LengthȻ��ݼ���֤�����ڶ�����Ч��ֵ */
      where_num %= Length; 
    }
    
  	float out()
    {
      if(flag>0)
        return now_num;
      else
        return (sum/Length);
    }
  private:
  	float buffer_num[Length];
	float now_num;
	float sum; 						/*<! ���Ⱥ����ֺ� */
	int flag,where_num;
};

/* IIRLowPassFilter */
class IIRLowPassFilter
{
public:
  /**
    @brief trust (0,1) 
   */
	IIRLowPassFilter(float trust = 1): Trust(trust)
  {
    last_out_num = 0;
  } 
  ~IIRLowPassFilter(){};
  float Trust;
  float f(float num);
protected:
  void in(float num);
  float out();
private:
  float in_num;
  float out_num;
  float last_out_num;
};

#endif

#endif

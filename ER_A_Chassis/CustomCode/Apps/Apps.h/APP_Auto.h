#ifndef __APP_AUTO_H
#define __APP_AUTO_H

#include "include.h"
#include "DEV_Posture.h"
#include "DEV_Analog.h"
enum AUTO_Mode_e
{
    AUTO_DisablelMode,
    AUTO_TestMode,
    AUTO_SemiAutolMode,
    AUTO_AutoGameMode
}; 
enum AUTO_Step_e
{
    AUTO_Origin = 0,
    AUTO_TakeRing_PointOne = 1,
    AUTO_TakeRing_PointTwo = 2,
    AUTO_TakeRing = 3,
    AUTO_ShootRing_PointOne = 4,
    AUTO_Aim_One = 5,
    AUTO_Revise_One = 6,
    AUTO_Shoot_One = 7,
    AUTO_ShootRing_PointTwo = 8,
    AUTO_Aim_Two = 9,
    AUTO_Revise_Two = 10,
    AUTO_Shoot_Two = 11,
    AUTO_ShootRing_PointThree = 12,
    AUTO_Aim_Three = 13,
    AUTO_Revise_Three = 14,
    AUTO_Shoot_Three = 15,
    AUTO_ShootRing_PointFour = 16,
    AUTO_Aim_Four = 17,
    AUTO_Revise_Four = 18,
    AUTO_Shoot_Four = 19,
    AUTO_ShootRing_PointFive = 20,
    AUTO_Aim_Five = 21,
    AUTO_Revise_Five = 22,
    AUTO_Shoot_Five = 23,
		AUTO_ShootRing_PointSix = 24,
    AUTO_Aim_Six = 25,
    AUTO_Revise_Six = 26,
    AUTO_Shoot_Six = 27,

    AUTO_Other = 28,
    AUTO_Test = 29,
};

class Auto_classdef
{
private:
    AUTO_Mode_e Mode=AUTO_TestMode;
    AUTO_Step_e step=AUTO_Origin;
    int point_time;
		int WallTime;
    float LR_time = 0;
    float LR_out_time = 0;

    // AUTO_CtrlMode_e mode;
		//5650 1000{730,340, 50,\
                370, 1000, 0,\
                -0,-0, 0};
    int Spots[3][3] = {-5200,-0, 0,\
                -5100, -100, 50,\
                -0,-0, 0};
		
		int Loop_Point_One[3]={-5881,608,0}, Loop_Point_Two[3], Loop_Point_Three[3], Loop_Point_Four[3];
		int Batting_Point_One[3]={-3546,870,0},\
        Batting_Point_Two[3]={-358,870,0}, \
        Batting_Point_Three[3]={2830,870,0 }, \
        Batting_Point_Four[3]={764,870,0 }, \
        Batting_Point_Five[3]={-1866,870,0 },\
        Batting_Point_Six[3]={-358,870,0 };
		
		void Point_Run(int point[3],AUTO_Step_e AUTO_step);
    void MicroWall(int dir,AUTO_Step_e AUTO_step);
    void Disable(void);
    void Manual(void);
    void SemiAuto(void);
    void Auto(void);
    void Left_PickIdea(void);
    void Front_PlaceIdea(void);

		uint8_t Detection_Point_text(int *Spots);
		bool IO_RTX(int io_s,int is_t);
public:
		uint8_t Posture_ResFlag;
    Posture_Classdef Posture;
		Analog_Classdef Analog = Analog_Classdef(1);

		int CH_F_X=1,CH_HL_X = 1,CH_HR_X = 1;

    uint8_t startFlag;//Æô¶¯
    uint8_t overFlag;//´ïµ½
		uint8_t SX;
		uint8_t text_step;
		float Vx,Vy,Vw;
		uint8_t WallFlag;
    
		void Game_Step();
		void Text_Step(void);
    void Process(void);
    uint8_t Detection_Point(int *Spots);
};


#endif
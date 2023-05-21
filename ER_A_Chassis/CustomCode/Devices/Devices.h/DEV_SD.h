//#ifndef __DEV_SD_H
//#define __DEV_SD_H

//#include "sdio.h"
////#include "fatfs.h"
//#include "DEV_RTC.h"

//#define SD_Create 0

//#define SD_State SD_Close
//#define SD_Open 1
//#define SD_Close 0


//typedef enum
//{
//    SD_ERR_NOT,
//    SD_ERR_MOUNT_MKFS,
//    SD_ERR_OPEN
//}SD_ERR_e;


//class SD_Classdef// : public RTC_Classdef
//{
//public:
//	SD_Classdef(){};
//		
//    SD_ERR_e SD_err;
//    static void read_ok(void);
//    void error_detect(uint8_t err); 

//    uint8_t Reset_Number_Init(void);
//    void Data_Save_Init(void);
//    void Data_Save(void);


//    void Set_DataNumber(int num){number_Data = num;}
//    void Set_DataName(char *ch,float *Dname);
//private:
//			
//    FATFS SDFatFs;                               /* file system object for SD card logical drive */
//    uint8_t buffer[_MAX_SS];                       /* a work buffer for the f_mkfs() */
//    uint32_t byteswritten, bytesread;               /* file write/read counts */
//    uint8_t rtext[100];                            /* file read buffer */


//    const char *f_name = "Reset.txt";           /* file name */

//    char W_Data_Buf[16];
//    char RW_mBuf[512];
//    float *DataSave[8];
//    int number_Data = 0;
//};

//#endif
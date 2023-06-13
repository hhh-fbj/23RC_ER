//#include "DEV_SD.h"


//#include "bsp_driver_sd.h"
//#include "stm32f4xx_hal.h"
//#include "fatfs.h"
//#include "main.h"

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>




///**
//  * @brief      show errors by blinking led
//  * @param[in]  err: error id
//  * @retval 
//  */
//void SD_Classdef::error_detect(uint8_t err)
//{
//		switch (err)
//		{
//			case SD_ERR_MOUNT_MKFS:
//			{
//				HAL_Delay(10);	
//			}break;

//			case SD_ERR_OPEN:
//			{
//				HAL_Delay(10);	
//			}break;
//			
//			default:
//			{
//				HAL_Delay(10);	
//			}break;
//		}
//}

///**
//  * @brief  doing some tests to SD card, like mount, create file, open a text etc. 
//  * @param  
//  * @retval 
//  */
//// int read_math;
//// char RW_mBuf[256];
//// FRESULT xb;
//// FRESULT xc;
//// uint8_t SD_Classdef::Reset_Number_Init(char * RW_Buf)
//// {
//// 	uint8_t err = 0;
//// 	const char *name_reset = "Reset.txt";
//// 	const char *_csv = ".csv";
//// 	if(BSP_SD_IsDetected() == SD_PRESENT)
//// 	{
//// //		//连接底层驱动
//// 		FATFS_LinkDriver(&SD_Driver,SDPath);
//// 		if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
//// 		{
//// 			if(f_open(&SDFile, name_reset, FA_CREATE_NEW) == FR_OK)
//// 			{
//// 				f_close(&SDFile);
//// 				if(f_open(&SDFile, name_reset, FA_WRITE) == FR_OK)
//// 				{
//// 					f_write(&SDFile, "1", sizeof("1"), &byteswritten);
//// 					f_close(&SDFile);
//// 					sprintf(RW_Buf, "%s", "1.csv");
//// 					err = 0;
//// 					return err;
//// 				}
//// 			}
//// 			else
//// 			{
//// 				if(f_open(&SDFile, name_reset, FA_READ) == FR_OK)  
//// 				{
//// 					xb = f_read(&SDFile, RW_Buf, sizeof(RW_Buf), (UINT*)&bytesread); 
//// 					f_close(&SDFile);
//// 					if(f_size(&SDFile) == 0)
//// 					{
//// 						#if	SD_Create				
//// 						if (f_mkfs((TCHAR const*)SDPath, FM_FAT32, 0, buffer, sizeof(buffer)) == FR_OK)
//// 						{
//// 							if(f_open(&SDFile, name_reset, FA_CREATE_ALWAYS|FA_WRITE) == FR_OK)
//// 							{
//// 								f_write(&SDFile, "1", sizeof("1"), &byteswritten);
//// 								f_close(&SDFile);
//// 								sprintf(RW_Buf, "%s", "1.csv");
//// 								err = 0;
//// 								return err;
//// 							}
//// 							else
//// 							{
//// 								err = 1;
//// 								return err;
//// 							}
//// 						}
//// 						else
//// 						{
//// 							err = 1;
//// 							return err;
//// 						}
//// 						#endif
//// 					}
//// 					read_math = atoi(RW_Buf);
//// 					read_math++;
//// 					sprintf(RW_Buf, "%d", read_math);
//// 					if (f_open(&SDFile, name_reset, FA_WRITE ) == FR_OK)  
//// 					{
//// 						/* Write data to the text file */
//// 						f_write(&SDFile, RW_Buf, sizeof(RW_Buf), &byteswritten);
//// 						/* Close the open text file */
//// 						f_close(&SDFile);
//// 						/* Open the text file object with read access */
//// 						err = 0;
//// 						strcat(RW_Buf,_csv);

//// 					}	
//// 					else
//// 					{
//// 						err = 1;
//// 					}
//					
//// 				}
//// 				else
//// 				{
//// 					err = 1;
//// 				}
//// 			}
//			
//// 		}
//// 		else
//// 		{
//// 			err = 1;
//// 		}
//// 	}
//// 	else
//// 	{
//// 		f_mount(NULL, (TCHAR const*)SDPath, 0);/* 注销文件系统对象 */
//// 		err = 2;
//// 	}
//	
//// 	return err; 
//// }

//// void SD_Classdef::Data_Save_Init(void)
//// {
//// 	char RW_Buf[16] = {0};
//// 	if(Reset_Number_Init(RW_Buf) == 0)
//// 	{
//// 		if(f_open(&SDFile, (const char*)RW_Buf, FA_CREATE_NEW) == FR_OK)
//// 		{
//// 			f_close(&SDFile);
//// 			if(f_open(&SDFile,(const char*)RW_Buf, FA_WRITE) == FR_OK)
//// 			{
//// 				memset(RW_mBuf,0,sizeof(RW_mBuf));
//// 				if(f_size(&SDFile) == 0)//标题
//// 				{
//// 					sprintf((char *)RW_mBuf,"%s,%s,%s\r\n","Time","SysStep","ErrCode");
//// 					if(f_write(&SDFile, RW_mBuf, sizeof(RW_mBuf),&byteswritten) == FR_OK)
//// 					{
//// 						f_close(&SDFile);
//// 					}
//// 					else
//// 					{

//// 					}
//// 				}
//// 		}
//// 		}
//// 		else
//// 		{
//			
//// 		}

//// 	}
//// 	else
//// 	{

//// 	}
//// 	FATFS_UnLinkDriver(SDPath);  /* 解除底层连接,否则不能实现热插拔 */
//// }






//int read_math;


//char RW_Buf[16] = {0};
//FRESULT xb;
//FRESULT xc;
//uint8_t SD_Classdef::Reset_Number_Init(void)
//{
//	uint8_t err = 0;
//	const char *name_reset = "Reset.txt";
//	const char *_csv = ".csv";
//	if(BSP_SD_IsDetected() == SD_PRESENT)
//	{
////		//连接底层驱动
//		FATFS_LinkDriver(&SD_Driver,SDPath);
//		if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
//		{
//			
//			if(f_open(&SDFile, (const TCHAR*)name_reset, FA_CREATE_NEW) == FR_OK)
//			{
//				f_close(&SDFile);
//				if(f_open(&SDFile, (const TCHAR*)name_reset, FA_WRITE) == FR_OK)
//				{
//					f_puts((const TCHAR*)RW_Buf, &SDFile);
//					f_close(&SDFile);
//					if (f_open(&SDFile, (const TCHAR*)name_reset, FA_OPEN_APPEND|FA_WRITE ) == FR_OK)  
//					{
//						f_puts((const TCHAR*)"\nxxx  \0", &SDFile);
//						f_puts((const TCHAR*)RW_Buf, &SDFile);
//						f_close(&SDFile);
//						err = 0;
//						strcat(RW_Buf,_csv);
//					}	
//				}
//			}
//			else
//			{
//				if(f_open(&SDFile, (const TCHAR*)name_reset, FA_READ) == FR_OK)  
//				{
//					f_gets((TCHAR*)RW_Buf,sizeof(RW_Buf),&SDFile);
//					// xb = f_read(&SDFile, RW_Buf, sizeof(RW_Buf), (UINT*)&bytesread); 
//					f_close(&SDFile);
//					if(f_size(&SDFile) == 0)
//					{
//						#if	SD_Create				
//						if (f_mkfs((TCHAR const*)SDPath, FM_FAT32, 0, buffer, sizeof(buffer)) == FR_OK)
//						{
//							if(f_open(&SDFile, name_reset, FA_WRITE) == FR_OK)
//							{
//								f_puts("1", &SDFile);
//								f_close(&SDFile);
//								if (f_open(&SDFile, name_reset, FA_OPEN_APPEND|FA_WRITE ) == FR_OK)  
//								{
//									f_puts("\nxxx  \0", &SDFile);
//									f_puts("1", &SDFile);
//								}
//								sprintf(RW_Buf, "%s", "1.csv");
//								err = 0;
//								return err;
//							}
//							else
//							{
//								err = 1;
//								return err;
//							}
//						}
//						else
//						{
//							err = 1;
//							return err;
//						}
//						#endif
//					}
//					read_math = atoi(RW_Buf);
//					read_math++;
//					sprintf(RW_Buf, "%d", read_math);
//					if (f_open(&SDFile, (const TCHAR*)name_reset, FA_WRITE ) == FR_OK)
//					{
//						f_puts((const TCHAR*)RW_Buf, &SDFile);
//						f_close(&SDFile);
//						if (f_open(&SDFile, (const TCHAR*)name_reset, FA_OPEN_APPEND|FA_WRITE ) == FR_OK)  
//						{
//							f_puts((const TCHAR*)"\nxxx  \0", &SDFile);
//							f_puts((const TCHAR*)RW_Buf, &SDFile);
//							f_close(&SDFile);
//							err = 0;
//							strcat(RW_Buf,_csv);
//						}
//					}
//					else
//					{
//						err = 1;
//					}
//					
//				}
//				else
//				{
//					err = 1;
//				}
//			}
//		}
//		else
//		{
//			err = 1;
//		}
//	}
//	else
//	{
//		f_mount(NULL, (TCHAR const*)SDPath, 0);/* 注销文件系统对象 */
//		err = 2;
//	}
//	
//	return err; 
//}

//void SD_Classdef::Data_Save_Init(void)
//{
//	if(Reset_Number_Init() == 0)
//	{
//		if(f_open(&SDFile, (const TCHAR*)RW_Buf, FA_CREATE_NEW) == FR_OK)
//		{
//			f_close(&SDFile);
//		}
//		else
//		{
//			
//		}
//		
//	}
//	else
//	{

//	}
////	FATFS_UnLinkDriver(SDPath);  /* 解除底层连接,否则不能实现热插拔 */
//}


//void SD_Classdef::Data_Save(void)
//{
////	FATFS_LinkDriver(&SD_Driver,SDPath);
//	xc = f_open(&SDFile, (const TCHAR*)RW_Buf, FA_WRITE|FA_OPEN_APPEND);
//	if(xc == FR_OK)
//	{
//		if(f_size(&SDFile) == 0)//标题
//		{
//			sprintf((char *)RW_mBuf,"%s\n",RW_mBuf);
//			f_puts((const TCHAR*)RW_mBuf, &SDFile);
//		}
//		else
//		{
//			memset(RW_mBuf,0,sizeof(RW_mBuf));
//			for(int i = 0;i < number_Data;i++)//
//			{
//				sprintf((char *)W_Data_Buf,"%.2lf,",DataSave[number_Data]);
//				strcat(RW_mBuf,W_Data_Buf);
//			}
//			sprintf((char *)RW_mBuf,"%s\n",RW_mBuf);
//			f_puts((const TCHAR*)RW_mBuf, &SDFile);
//		}
//		f_close(&SDFile);
//	}
//	else
//	{
//		// sprintf((char *)RW_mBuf,"%s,%s,%s\r\n","ttt","sss","eee");
//		// f_puts((const TCHAR*)RW_mBuf, &SDFile);
//		// if(f_write(&SDFile, RW_mBuf, sizeof(RW_mBuf),&byteswritten) == FR_OK)
//		// {
//			
//		// }
//		// else
//		// {
//			
//		// }
//		// f_close(&SDFile);
//	}
////	FATFS_UnLinkD1-river(SDPath); 
//	/* 分组追加运行数据 */
////	for(int i=0;i<0/*Data_Size*/;i++)
////	{
////		memset(RW_Buf,0,sizeof(RW_Buf));
//////		sprintf();
////		if(f_write(&SDFile, RW_Buf, sizeof(RW_Buf),&byteswritten) == FR_OK)
////		{

////		}
////		else
////		{

////		}
////	}
//}

// void SD_Classdef::Set_DataName(char *ch,float *Dname) 
//{
//	DataSave[number_Data] = Dname;
//	if(number_Data != 0)
//	{
//		sprintf((char *)RW_mBuf,"%s,",RW_mBuf);
//	}
//	strcat(RW_mBuf, ch);
//	
//	number_Data++;
//}


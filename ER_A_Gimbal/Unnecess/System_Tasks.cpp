
/* Includes ------------------------------------------------------------------*/
#include "System_Tasks.h"
#include "System_DataPool.h"
#include "Task_ALL.h"
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/




/* Function prototypes -------------------------------------------------------*/

/**
  * @brief      创建用户任务
  * @param[in]  None
  * @return     None
  */
void System_Tasks_Init(void)
{
    Infantry.State = Robot_Initializing;

    xTaskCreate((TaskFunction_t )Task_CAN1_Receive,             
                (const char*    )"CAN1_Receive",   
                (uint16_t       )256,
                (void*          )NULL,
                (UBaseType_t    )PriorityHigh,
				(TaskHandle_t*  )NULL);


    xTaskCreate((TaskFunction_t )Task_CAN2_Receive,             
                (const char*    )"CAN2_Receive",   
                (uint16_t       )256,
                (void*          )NULL,
                (UBaseType_t    )PriorityHigh,
				(TaskHandle_t*  )NULL);

    // xTaskCreate((TaskFunction_t )Task_DevicesMonitor,             
    //             (const char*    )"DevicesMonitor",   
    //             (uint16_t       )128,
    //             (void*          )NULL,
    //             (UBaseType_t    )PriorityNormal,
	// 			(TaskHandle_t*  )NULL);


    xTaskCreate((TaskFunction_t )Task_DataSampling,             
                (const char*    )"IMUSampling",   
                (uint16_t       )512,
                (void*          )NULL,
                (UBaseType_t    )PriorityHigh,
                (TaskHandle_t*  )NULL);

    // xTaskCreate((TaskFunction_t )Task_Debug,             
    //             (const char*    )"Debug",   
    //             (uint16_t       )256,
    //             (void*          )NULL,
    //             (UBaseType_t    )PriorityAboveNormal,
    //             (TaskHandle_t*  )NULL);


    // xTaskCreate((TaskFunction_t )Recv_Referee,             
    //             (const char*    )"Referee",   
    //             (uint16_t       )512,
    //             (void*          )NULL,
    //             (UBaseType_t    )PriorityAboveNormal,
	// 			(TaskHandle_t*  )&RecvReferee_Handle);

    xTaskCreate((TaskFunction_t )Task_Control,             
                            (const char*    )"Control",   
                            (uint16_t       )640,
                            (void*          )NULL,
                            (UBaseType_t    )PriorityHigh,
            (TaskHandle_t*  )NULL);


    Infantry.State = Robot_Activating;
}
















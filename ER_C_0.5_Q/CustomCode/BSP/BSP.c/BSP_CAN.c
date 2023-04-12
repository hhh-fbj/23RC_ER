
#include "BSP_CAN.h"
/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static void (*pCAN1_RxCpltCallback)(CanRxMsg_t*);
static void (*pCAN2_RxCpltCallback)(CanRxMsg_t*);
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
* @brief  Initialize CAN Bus
* @param  hcan: CANx created by CubeMX.
* @return None.
*/
void CAN_Init(CAN_HandleTypeDef* hcan, void (*pFunc)(CanRxMsg_t*))
{
	/* Check the parameters */
	assert_param(hcan != NULL);

	HAL_CAN_Start(hcan);
	  

	if (hcan->Instance == CAN1)
	{
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); 
		pCAN1_RxCpltCallback = pFunc;
		
	}
	else if (hcan->Instance == CAN2)
	{
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
		pCAN2_RxCpltCallback = pFunc;
		
	}
}



void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef  CAN_FilterInitStructure;
	/* Check the parameters */
	assert_param(hcan != NULL);



	if (hcan->Instance == CAN1)
	{
		CAN_FilterInitStructure.FilterActivation     = ENABLE;
		CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
		CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
		CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
		CAN_FilterInitStructure.FilterMaskIdLow      = 0x0300;
		CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
		CAN_FilterInitStructure.FilterIdLow          = 0x0300;
		CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO1;
		
		CAN_FilterInitStructure.SlaveStartFilterBank = 0;
		CAN_FilterInitStructure.FilterBank = 0;
		HAL_CAN_ConfigFilter(hcan, &CAN_FilterInitStructure);
	}
	else if(hcan->Instance == CAN2)
	{
		CAN_FilterInitStructure.FilterActivation     = ENABLE;
		CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
		CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
		CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
		CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
		CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
		CAN_FilterInitStructure.FilterIdLow          = 0x0000;
		CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
		
		CAN_FilterInitStructure.SlaveStartFilterBank = 14;
		CAN_FilterInitStructure.FilterBank = 14;
		HAL_CAN_ConfigFilter(hcan, &CAN_FilterInitStructure);
	}

}


/**
* @brief  Send an communication frame by CAN.
* @param  hcan  :CAN bus used to send.
* @param  ID    :ID of frame.
* @param  *pData:Data to send.
* @param  Len   :Length of data.
* @return CAN_SUCCESS:  Operation success.
* @return CAN_LINE_BUSY:CAN line busy.
*/
uint8_t CANx_SendData(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t* pData, uint16_t Len)
{
	static CAN_TxHeaderTypeDef Tx_Header;
	uint32_t used_mailbox;
	/* Check the parameters */
	assert_param(hcan != NULL);

	Tx_Header.StdId = ID;
	Tx_Header.ExtId = ID;
	Tx_Header.IDE = 0;
	Tx_Header.RTR = 0;
	Tx_Header.DLC = Len;

	if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, &used_mailbox) != HAL_OK)
	{
		return CAN_LINE_BUSY;
	}
	else {}

	return CAN_SUCCESS;
}

uint8_t CANx_SendData_EXT(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t* pData, uint16_t Len)
{
	static CAN_TxHeaderTypeDef Tx_Header;
	uint32_t used_mailbox;
	/* Check the parameters */
	assert_param(hcan != NULL);

	// Tx_Header.StdId = ID;
	Tx_Header.ExtId = ID;
	Tx_Header.IDE = 4;
	Tx_Header.RTR = 0;
	Tx_Header.DLC = Len;

	if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, &used_mailbox) != HAL_OK)
	{
		return CAN_LINE_BUSY;
	}
	else {}

	return CAN_SUCCESS;
}





/*HAL库FIFO0中断*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	/*!< CAN receive buffer */
	static CanRxMsg_t CAN_RxBuffer;

	/* Switch to user call back function. */
	if (hcan->Instance == CAN1)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN_RxBuffer.Header, CAN_RxBuffer.Data) == HAL_ERROR) {};
		pCAN1_RxCpltCallback(&CAN_RxBuffer);
	}
	else if (hcan->Instance == CAN2)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN_RxBuffer.Header, CAN_RxBuffer.Data) == HAL_ERROR) {};
		pCAN2_RxCpltCallback(&CAN_RxBuffer);
	}
}




/*HAL库FIFO1中断*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	/*!< CAN receive buffer */
	static CanRxMsg_t CAN_RxBuffer;

	/* Switch to user call back function. */
	if (hcan->Instance == CAN1)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN_RxBuffer.Header, CAN_RxBuffer.Data) == HAL_ERROR) {};
		pCAN1_RxCpltCallback(&CAN_RxBuffer);
	}
	else if (hcan->Instance == CAN2)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN_RxBuffer.Header, CAN_RxBuffer.Data) == HAL_ERROR) {};
		pCAN2_RxCpltCallback(&CAN_RxBuffer);
	}
}



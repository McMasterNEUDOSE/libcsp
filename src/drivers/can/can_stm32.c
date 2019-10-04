/**
 * @file can_stm32.c
 * @author mostafa ayesh (ayeshm@mcmaster.ca)
 * @brief CSP CAN driver for STM32
 * @version 0.0
 * @date 2019-10-04
 */

/* STM32HAL Includes */
#include "stm32l4xx_hal.h"

/* FreeRTOS Includes */
#include <FreeRTOS.h>
#include <task.h>

/* CSP Includes */
#include <csp/csp.h>
#include <csp/arch/csp_thread.h>
#include <csp/interfaces/csp_if_can.h>

/* CAN Driver Includes */
#include "can.h"
#include "can_stm32.h"

/* Callback functions */
can_tx_callback_t txcb;
can_rx_callback_t rxcb;

/* STM32 CAN */
CAN_HandleTypeDef hcan1;
uint32_t TxMailbox;

/**
 * @brief CAN receive thread
 * 
 */
CSP_DEFINE_TASK(can_rx_thread)
{

    CAN_RxHeaderTypeDef rx_frame;
    uint8_t RxData[8];

    while (1)
    {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 1)
        {
            csp_log_info("RxFifo: empty");
            continue;
        }

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_frame, RxData) != HAL_OK)
        {
            csp_log_error("CANRx: error");
            continue;
        }

        can_frame_t frame;
        frame.dlc = rx_frame.DLC;
        frame.id = rx_frame.ExtId;
        for (int i = 0; i < frame.dlc; i++)
        {
            frame.data[i] = RxData[i];
        }
        if (rxcb != NULL)
            rxcb((can_frame_t *)&frame, NULL);
    }
    return CSP_TASK_RETURN;
}

/**
 * @brief Initializes CSP CAN Interface
 * 
 * @param id 
 * @param mask 
 * @param atxcb     CAN TX callback
 * @param arxcb     CAN RX callback
 * @param conf      CAN Config
 * @return int      0 for success, -1 for failure
 */
int can_init(uint32_t id, uint32_t mask, can_tx_callback_t atxcb, can_rx_callback_t arxcb, struct csp_can_config *conf)
{

    /* Set callbacks */
    txcb = atxcb;
    rxcb = arxcb;

    /* Set the interface up */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan1);

    /* Configure filter 
     * Accepts any ID
     */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        csp_log_error("CAN Filter Error\n");
        return -1;
    }

    /* Start CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        csp_log_error("CANSTART: error");
    }

    /* Create receive thread */
    csp_thread_handle_t can_rx_handle;
    csp_thread_create(can_rx_thread, "CAN RX", 1000, NULL, 0, &can_rx_handle);

    return 0;
}

/**
 * @brief Transmits a CAN frame
 * 
 * @param id                CAN message id
 * @param data              Data packet 
 * @param dlc               Data length code
 * @param task_woken        ISR related
 * @return int              Status code (0 success, -1 fail)
 */
int can_send(can_id_t id, uint8_t data[], uint8_t dlc, CSP_BASE_TYPE *task_woken)
{
    CAN_TxHeaderTypeDef tx_frame;
    uint8_t TxData[8];

    int i, tries = 0;

    if (dlc > 8)
        return -1;

    /* Copy identifier (Extended) */
    tx_frame.ExtId = id;
    tx_frame.RTR = CAN_RTR_DATA;
    tx_frame.IDE = CAN_ID_EXT;

    tx_frame.TransmitGlobalTime = DISABLE;

    /* Copy data to frame */
    for (i = 0; i < dlc; i++)
        TxData[i] = data[i];

    /* Set DLC */
    tx_frame.DLC = dlc;

    /* Send frame */
    HAL_CAN_AddTxMessage(&hcan1, &tx_frame, TxData, &TxMailbox);

    /* Wait for transmission to complete */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
    {
        if (++tries < 1000)
        {
            /* Wait 10 ms and try again */
            csp_sleep_ms(10);
        }
        else
        {
            csp_log_error("write: error");
            break;
        }
    }

    return 0;
}

/**
 * @brief MSP Initializer for CAN
 * 
 * @param canHandle 
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1)
    {
        /* CAN1 clock enable */
        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**CAN1 GPIO Configuration    
        PA11     ------> CAN1_RX
        PA12     ------> CAN1_TX 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief MSP Deinitializer for CAN
 * 
 * @param canHandle 
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

    if (canHandle->Instance == CAN1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN1 GPIO Configuration    
        PA11     ------> CAN1_RX
        PA12     ------> CAN1_TX 
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    }
}
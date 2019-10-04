
/**
 * @file can_stm32.h
 * @author mostafa ayesh (ayeshm@mcmaster.ca)
 * @brief CSP CAN driver for STM32
 * @version 0.0
 * @date 2019-10-01
 */
#ifndef __CAN_STM32_H__
#define __CAN_STM32_H__

#ifdef __cplusplus
extern "C"
{
#endif

// #define CAN_LOOPBACK // Enables CAN loopback for testing

#ifdef CAN_LOOPBACK
#define CAN_MODE CAN_MODE_LOOPBACK
#else
#define CAN_MODE CAN_MODE_NORMAL
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CAN_STM32_H__ */

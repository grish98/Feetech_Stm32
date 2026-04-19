/*
Application Test Code for STM32F104CBT6 Dev Board Using UART 2 (UART 1 damaged, verified with oscilsicope)
+ Waveshare Bus Servo Adapter
1000000 Baud rate.
No paritity Bits
1 stop bit
Word Length 8 bits
Full Duplex
Note: if not using Wave Share Bus Servo adapter, would need to be in half duplex, because these servo have one wire for both rx adn tx (which the adapter automatically handles)
*/

#include "sts_protocol.h"
#include "sts_servo.h"
#include "sts_registers.h"
#include "sts_servo_cmd.h"
#include "sts_ports_stm32.h"
#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart2;


void AppMain(void) {
  
static sts_bus_t servo_bus = {0};
static sts_servo_t servo_1 = {0};

  STS_Bus_Init(&servo_bus, &huart2, STM32_UART_Transmit, STM32_UART_Receive);
  STS_Servo_Init(&servo_1, &servo_bus, 2);

  while (1)
  {
    sts_result_t res = STS_servo_ping(&servo_1); 
    if (res == STS_OK || servo_1.is_online == STS_ONLINE)
    {
      //fast blink for success
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      //movement test
      uint16_t pos1= 2048;
      STS_SetTargetPosition(&servo_1, pos1);
      HAL_Delay(1000); 
      STS_SetTargetPosition(&servo_1, 0);
      HAL_Delay(1000); 
    }
    else
    
    {
      // Slow blink for failure
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
        HAL_Delay(2000); 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   
        HAL_Delay(2000);
    }
  }
}
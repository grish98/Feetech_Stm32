/*
Application Test Code for STM32F104CBT6 Dev Board Using UART 2 (UART 1 damaged, verified with oscilloscope)
1000000 Baud rate. No parity. 1 stop bit. 8-bit word length.

Wiring mode — select via STM32_UART_SetHalfDuplex() below:
  Half-duplex (enabled=1): direct servo wiring. Servo DATA → PA2 (TX pin). STM32 HDSEL
    mode shares the TX pin for both directions; no adapter or resistor needed.
  Full-duplex (enabled=0): Waveshare Bus Servo Adapter. Adapter converts the STM32
    full-duplex UART to the servo's single-wire half-duplex protocol.
*/

#include "sts_protocol.h"
#include "sts_servo.h"
#include "sts_registers.h"
#include "sts_servo_cmd.h"
#include "sts_ports_stm32.h"
#include "main.h"
#include <stdint.h>
#include "Hw_Tests.h"
#include "SEGGER_RTT.h"

extern UART_HandleTypeDef huart2;


void AppMain(void) {

static sts_bus_t servo_bus = {0};
static sts_servo_t servo_1 = {0};

  STS_Bus_Init(&servo_bus, &huart2, STM32_UART_Transmit, STM32_UART_Receive);
  servo_bus.flush_rx    = STM32_UART_FlushRx;
  servo_bus.max_retries = 0U;  /* no retries during scan */
  STM32_UART_SetHalfDuplex(&huart2, 1);  /* 1 = direct wiring (half-duplex), 0 = Waveshare adapter */

  /* ID scan — any non-timeout response means the servo heard us. */
  SEGGER_RTT_printf(0, ">> Scanning IDs 1-15...\n");
  uint8_t found_id = 0U;
  for (uint8_t scan_id = 1U; scan_id <= 15U; scan_id++) {
      STS_Servo_Init(&servo_1, &servo_bus, scan_id);
      sts_result_t scan_res = STS_servo_ping(&servo_1);
      SEGGER_RTT_printf(0, "   ID %d: err=%d\n", (int)scan_id, (int)scan_res);
      if (scan_res != STS_ERR_TIMEOUT) {  /* any response (even malformed) = servo found */
          found_id = scan_id;
          break;
      }
      HAL_Delay(10);
  }
  if (found_id == 0U) {
      SEGGER_RTT_printf(0, ">> No servo found on IDs 1-15. Check wiring and power.\n");
      while (1) { HAL_Delay(1000); }
  }
  SEGGER_RTT_printf(0, ">> Servo found at ID %d\n", (int)found_id);

  servo_bus.max_retries = 2U;
  STS_Servo_Init(&servo_1, &servo_bus, found_id);

  STS_SetTorqueEnable(&servo_1, 1);
  HAL_Delay(50);

  STS_RunStressTest(&servo_1, 200U);

}
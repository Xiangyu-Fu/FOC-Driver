/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *    Copyright (c) 2023 fortiss GmbH, Xiangyu Fu
 *    All rights reserved.
 */
#include <Arduino.h>

#include "interface_task.h"
#include "motor_task.h"
#include "uart_task.h"
#include "display_task.h"

#include "SPI.h"
#include "TFT_eSPI.h"

static MotorTask motor_task(1);
static UARTTask uart_task(0);
static DisplayTask display_task(0);
InterfaceTask interface_task(0, motor_task, uart_task);

extern TFT_eSPI tft;

void setup()
{
  // UART setup
  Serial.begin(115200);

  motor_task.setLogger(&interface_task);
  motor_task.begin();
  interface_task.begin();
  uart_task.begin();
  display_task.begin();

  // Free up the Arduino loop task
  vTaskDelete(NULL);
}

void loop()
{

}
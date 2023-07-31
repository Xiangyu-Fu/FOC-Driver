/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *    Copyright (c) 2023 fortiss GmbH, Xiangyu Fu
 *    All rights reserved.
 */
#include "uart_task.h"
#include <HardwareSerial.h>


#define BUF_SIZE (128)

HardwareSerial SerialPort(2); // use UART2

UARTTask::UARTTask(const uint8_t task_core) : Task("UART Task", 8192, 0, task_core) {
    SerialPort.begin(115200, SERIAL_8N1, 16, 17);
    // uart_queue_ = xQueueCreate(1, sizeof(UartConfig));
}

UARTTask::~UARTTask() {
    // Destructor code here
}

void UARTTask::addListener(QueueHandle_t queue) {
    listeners_.push_back(queue);
}

void UARTTask::setLogger(Logger* logger) {
    logger_ = logger;
}

// publish is used for the logger
void UARTTask::publish(const PB_SmartKnobConfig & config) {
    for (auto listener : listeners_) {
        xQueueOverwrite(listener, &config); // xQueueOverwrite will overwrite the queue if it is full
    }
}

void UARTTask::run() {
    uint8_t data[BUF_SIZE];
    Serial.println("UART ready!");

    for (;;) {
        
        if (SerialPort.available())
        {
            // read the whole sentence
            int len = SerialPort.readBytesUntil('\n', (char*)data, BUF_SIZE);
            if (len > 95) {
                if(messageProcess((char*)data) == 0){
                    // // For debug
                    // Serial.printf("Get new configurations from UART, ");
                    // Serial.printf("received %d bytes \n", len);
                    data[len] = '\0';
                    publish(uartConfig);
                }
                else{
                    Serial.printf("Parsing ERROR: %s \n", data);
                }
            }
            else{
                Serial.printf("Invalid message: %s \n", data);
            }
        vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

int UARTTask::messageProcess(char * uart_string) {
    if (uart_string == NULL) {
        return -1;
    }

    sscanf(uart_string, "ns: %d, pn: %d, posidians: %f, dete_unit: %f, enth_unit: %f, snapoint: %f",
    &uartConfig.num_positions, &uartConfig.position, &uartConfig.position_width_radians,
    &uartConfig.detent_strength_unit, &uartConfig.endstop_strength_unit, &uartConfig.snap_point);

    strcpy(uartConfig.text, "New Configurations from UART");
    
    return 0;
}


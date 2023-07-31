/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *    Copyright (c) 2023 fortiss GmbH, Xiangyu Fu
 *    All rights reserved.
 */
#pragma once
#include <Arduino.h>
#include <vector>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "logger.h" 
#include "task.h"
#include "proto_gen/smartknob.pb.h"

class UARTTask : public Task<UARTTask> {
    friend class Task<UARTTask>; // Allow base Task to invoke protected run()

public:
    UARTTask(const uint8_t task_core);
    ~UARTTask();

    void addListener(QueueHandle_t queue);
    void setLogger(Logger* logger);
    void publish(const PB_SmartKnobConfig & config);
    
protected:
    void run();

private:
    // QueueHandle_t uart_queue_;
    Logger* logger_;
    std::vector<QueueHandle_t> listeners_;
    PB_SmartKnobConfig uartConfig; // Note: Here use the instance instead of pointer, because pointer will point to NULL

    void onRequest();
    void onReceive(int numBytes);
    void processMessage(char* message, int length);
    int  messageProcess(char* uart_string);

    char buf_[72];
};


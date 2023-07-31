/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *    Copyright (c) 2023 fortiss GmbH, Xiangyu Fu
 *    All rights reserved.
 */
#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <vector>

#include "logger.h"
#include "proto_gen/smartknob.pb.h"
#include "task.h"


enum class CommandType {
    CALIBRATE,
    CONFIG,
    HAPTIC,
};

struct HapticData {
    bool press;
};

struct Command {
    CommandType command_type;
    union CommandData {
        uint8_t unused;
        PB_SmartKnobConfig config;
        HapticData haptic;
    };
    CommandData data;
};

class MotorTask : public Task<MotorTask> {
    friend class Task<MotorTask>; // Allow base Task to invoke protected run()

    public:
        MotorTask(const uint8_t task_core);
        ~MotorTask();

        void setConfig(const PB_SmartKnobConfig& config);
        void playHaptic(bool press);
        // void runCalibration();

        void addListener(QueueHandle_t queue);
        void setLogger(Logger* logger);

    protected:
        void run();

    private:
        QueueHandle_t queue_;
        Logger* logger_;
        std::vector<QueueHandle_t> listeners_;
        char buf_[72];

        // BLDC motor & driver instance
        BLDCMotor motor = BLDCMotor(7, 11.2); // Motor pairs, phase resistance
        // BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
        BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

        void publish(const PB_SmartKnobState& state);
        // void calibrate();
        void checkSensorError();
        void log(const char* msg);
};

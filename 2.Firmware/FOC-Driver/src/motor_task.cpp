/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *    Copyright (c) 2023 fortiss GmbH, Xiangyu Fu
 *    All rights reserved.
 */
#include <SimpleFOC.h>

#include "motor_task.h"
#if SENSOR_MT6701
#include "mt6701_sensor.h"
#endif
#if SENSOR_TLV
#include "tlv_sensor.h"
#endif
#include "util.h"


// #### 
// Hardware-specific motor calibration constants.
// Run calibration once at startup, then update these constants with the calibration results.
static const float ZERO_ELECTRICAL_OFFSET = 2.77;
static const Direction FOC_DIRECTION = Direction::CW;
static const int MOTOR_POLE_PAIRS = 7;
// ####


static const float DEAD_ZONE_DETENT_PERCENT = 0.3;
static const float DEAD_ZONE_RAD = 2 * _PI / 180;

static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;


MotorTask::MotorTask(const uint8_t task_core) : Task("Motor", 2500, 1, task_core) {
    queue_ = xQueueCreate(5, sizeof(Command));
    assert(queue_ != NULL);
}

MotorTask::~MotorTask() {}

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

float target_angle = 5.0;
long timestamp_us = _micros();

void MotorTask::run(){

    // motor setup
    driver.voltage_power_supply = 12;
    driver.init();

    motor.linkDriver(&driver);

    // Initialize the I2C bus
    I2Cone.setPins(19, 18);
    encoder.init(&I2Cone);

    motor.controller = MotionControlType::torque;
    motor.voltage_limit = 5;
    motor.velocity_limit = 10000;
    motor.linkSensor(&encoder);


    // velocity control loop setup
    motor.PID_velocity.P = 0;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10000; 
    motor.PID_velocity.limit = 2;

    // velocity low pass filter time constant
    //motor.LPF_velocity.Tf = 0.01;

    // // angle control loop setup
    // motor.P_angle.P = 5;
    // motor.P_angle.I = 0;
    // motor.P_angle.D = 0;
    // // acceleration limit for angle control loop
    // motor.P_angle.output_ramp = 10000;

    // /* motion control limitation */
    // // maximal voltage for motor
    // motor.voltage_limit = 12;
    // // current limitation - if phase resistance is set
    // motor.current_limit = 3;
    // // set the maximal velocity limit
    // motor.velocity_limit = 4;

    /* Calibration of motor and sensors*/
    motor.init();

    encoder.update(); // here is from the future version of SimpleFOC
    delay(10);

    motor.pole_pairs = MOTOR_POLE_PAIRS;
    motor.initFOC();
    // motor.initFOC(ZERO_ELECTRICAL_OFFSET, FOC_DIRECTION); // the calibration routine is called inside the init function
    // command.add('T', doTarget, "target velocity");
    
    motor.monitor_downsample = 0; // disable monitor at first - optional


    float current_detent_center = motor.shaft_angle;
    PB_SmartKnobConfig config = {
        .num_positions = 2,
        .position = 0,
        .position_width_radians = 60 * _PI / 180,
        .detent_strength_unit = 0,
    };

    float idle_check_velocity_ewma = 0;
    uint32_t last_idle_start = 0;
    uint32_t last_publish = 0;

    // PB_SmartKnobConfig latest_config = config;

    // UART setup
    // Serial.begin(115200);
    Serial.println("Motor ready!");
    // Serial.println("Set target velocity [rad/s]");
    while (1)
    {
        motor.loopFOC();

        // // print current position, after checking the sensor reading is work well
        // char str_angle[20];
        // sprintf(str_angle, "%f", encoder.getAngle());
        // log(str_angle);

        // Check queue for pending requests from other tasks
        Command command;
                if (xQueueReceive(queue_, &command, 0) == pdTRUE) {
            switch (command.command_type) {
                case CommandType::CALIBRATE:
                    break;
                case CommandType::CONFIG: {
                    // Change haptic input mode
                    config = command.data.config;
                    // latest_config = config;
                    log("Got new config");
                    current_detent_center = motor.shaft_angle;
                    #if SK_INVERT_ROTATION
                        current_detent_center = -motor.shaft_angle;
                    #endif

                    // Update derivative factor of torque controller based on detent width.
                    // If the D factor is large on coarse detents, the motor ends up making noise because the P&D factors amplify the noise from the sensor.
                    // This is a piecewise linear function so that fine detents (small width) get a higher D factor and coarse detents get a small D factor.
                    // Fine detents need a nonzero D factor to artificially create "clicks" each time a new value is reached (the P factor is small
                    // for fine detents due to the smaller angular errors, and the existing P factor doesn't work well for very small angle changes (easy to
                    // get runaway due to sensor noise & lag)).
                    // TODO: consider eliminating this D factor entirely and just "play" a hardcoded haptic "click" (e.g. a quick burst of torque in each
                    // direction) whenever the position changes when the detent width is too small for the P factor to work well.
                    const float derivative_lower_strength = config.detent_strength_unit * 0.0095;
                    const float derivative_upper_strength = config.detent_strength_unit * 0.002;
                    const float derivative_position_width_lower = radians(3);
                    const float derivative_position_width_upper = radians(8);
                    const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength)/(derivative_position_width_upper - derivative_position_width_lower)*(config.position_width_radians - derivative_position_width_lower);
                    // CLAMP is a template function that returns the value clamped to the range [lower, upper]
                    motor.PID_velocity.D = CLAMP(
                        raw,
                        min(derivative_lower_strength, derivative_upper_strength),
                        max(derivative_lower_strength, derivative_upper_strength)
                    );
                    char str[30];
                    sprintf(str, "%s%f", "motor.PID_velocity.P: ", motor.PID_velocity.P);
                    log(str);
                    sprintf(str, "%s%f%s", "motor.PID_velocity.D: ", motor.PID_velocity.D, "\n");
                    log(str);
                    break;
                }
                case CommandType::HAPTIC: {
                    // Play a hardcoded haptic "click"
                    float strength = command.data.haptic.press ? 5 : 1.5;
                    motor.move(strength);
                    for (uint8_t i = 0; i < 3; i++) {
                        motor.loopFOC();  // what is this for?
                        delay(1);
                    }
                    motor.move(-strength);
                    for (uint8_t i = 0; i < 3; i++) {
                        motor.loopFOC();
                        delay(1);
                    }
                    motor.move(0);
                    motor.loopFOC();
                    break;
                }
            }
        }

        /* HERE HAS SOME ISSUES, WHICH MAY CAUSE THE VIBRATION OF THE MOTOR */

        // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
        idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
        if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
            last_idle_start = 0;
        } else {
            if (last_idle_start == 0) {
                last_idle_start = millis();
            }
        }
        if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
            current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
        }


        // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
        float angle_to_detent_center = motor.shaft_angle - current_detent_center;
        #if SK_INVERT_ROTATION
            angle_to_detent_center = -motor.shaft_angle - current_detent_center;
        #endif
        if (angle_to_detent_center > config.position_width_radians * config.snap_point && (config.num_positions <= 0 || config.position > 0)) {
            current_detent_center += config.position_width_radians;
            angle_to_detent_center -= config.position_width_radians;
            config.position--;
        } else if (angle_to_detent_center < -config.position_width_radians * config.snap_point && (config.num_positions <= 0 || config.position < config.num_positions - 1)) {
            current_detent_center -= config.position_width_radians;
            angle_to_detent_center += config.position_width_radians;
            config.position++;
        } 

        //  the dead zone
        float dead_zone_adjustment = CLAMP(
            angle_to_detent_center,
            fmaxf(-config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
            fminf(config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

        bool out_of_bounds = config.num_positions > 0 && ((angle_to_detent_center > 0 && config.position == 0) || (angle_to_detent_center < 0 && config.position == config.num_positions - 1));
        motor.PID_velocity.limit = 2; //out_of_bounds ? 10 : 3;
        motor.PID_velocity.P = out_of_bounds ? config.endstop_strength_unit * 0.3 : config.detent_strength_unit * 0.3;

        float torqueMsg = 0;
        // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
        if (fabsf(motor.shaft_velocity) > 60) {
            // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
            motor.move(0);
        } else {
            float torque = motor.PID_velocity(-angle_to_detent_center + dead_zone_adjustment);
            #if SK_INVERT_ROTATION
                torque = -torque;
            #endif
            motor.move(torque); // Publish this torque to OPC UA
            torqueMsg = torque;
        }
        
        // // Log torque, for debug
        // char TorqueStr[10];
        // sprintf(TorqueStr, "%f", torqueMsg);
        // log(TorqueStr);

        // Publish current status to other registered tasks periodically
        if (millis() - last_publish > 5) {
            publish({
                .current_position = config.position,
                .sub_position_unit = -angle_to_detent_center / config.position_width_radians,
                .has_config = true,
                .current_force = torqueMsg,
                .config = config,
            });
            last_publish = millis();
        }

        motor.monitor();
        
        delay(1); 
    }
}


void MotorTask::setConfig(const PB_SmartKnobConfig& config) {
    Command command = {
        .command_type = CommandType::CONFIG,
        .data = {
            .config = config,
        }
    };
    xQueueSend(queue_, &command, portMAX_DELAY);
}


void MotorTask::playHaptic(bool press) {
    Command command = {
        .command_type = CommandType::HAPTIC,
        .data = {
            .haptic = {
                .press = press,
            },
        }
    };
    xQueueSend(queue_, &command, portMAX_DELAY);
}

// this addListener is used for the logger
void MotorTask::addListener(QueueHandle_t queue) {
    listeners_.push_back(queue); // push_back is a vector method
}

// publish is used for the logger
void MotorTask::publish(const PB_SmartKnobState& state) {
    for (auto listener : listeners_) {
        xQueueOverwrite(listener, &state); // xQueueOverwrite will overwrite the queue if it is full
    }
}

// not work because the macro is not defined
void MotorTask::checkSensorError() {
#if SENSOR_TLV
    if (encoder.getAndClearError()) {
        log("LOCKED!");
    }
#elif SENSOR_MT6701
    MT6701Error error = encoder.getAndClearError();
    if (error.error) {
        snprintf(buf_, sizeof(buf_), "CRC error. Received %d; calculated %d", error.received_crc, error.calculated_crc);
        log(buf_);
    }
#endif
}

void MotorTask::setLogger(Logger* logger) {
    logger_ = logger;
}

void MotorTask::log(const char* msg) {
    if (logger_ != nullptr) {
        logger_->log(msg);
    }
}

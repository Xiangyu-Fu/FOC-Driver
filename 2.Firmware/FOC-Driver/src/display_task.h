#pragma once
#include <Arduino.h>
#include <vector>
#include <Wire.h>

#include "SPI.h"
#include "logger.h"
#include "task.h"
#include "TFT_eSPI.h"

unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();

class DisplayTask : public Task<DisplayTask>
{
    friend class Task<DisplayTask>; // Allow base Task to invoke protected run()

public:
    DisplayTask(const uint8_t task_core);
    ~DisplayTask();

    void addListener(QueueHandle_t queue);
    void setLogger(Logger *logger);
    // void publish(const PB_SmartKnobConfig & config);

protected:
    void run();

private:
    // QueueHandle_t uart_queue_;
    Logger* logger_;
    std::vector<QueueHandle_t> listeners_;
    // PB_SmartKnobConfig uartConfig; // Note: Here use the instance instead of pointer, because pointer will point to NULL

    void onRequest();
    void onReceive(int numBytes);
    void processMessage(char *message, int length);
    int messageProcess(char *uart_string);

    char buf_[72];
};

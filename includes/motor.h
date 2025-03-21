#pragma once

#include <cstdint> // For uint8_t
#include <queue>
#include <functional>

#include "mbed.h"
#include "uartCOM.h"

/**
* @brief: Represents a single motor.
* 
* The motor is able to move in a certain direction and speed, for a number of steps.
* It is possible to configure some motor parameters like zero position etc.
*/
class Motor
{
    public:
    Motor(uint8_t address, EventQueue* evQueue);
    ~Motor();
    
    void Go(uint8_t dirspeed, uint32_t nbSteps);
    
    bool Calibrate();
    //bool SetZero();
    //bool GoToZero();
    
    bool SetPID(uint16_t kp, uint16_t ki, uint16_t kd);
    bool SetACC(uint16_t ACC);
    bool SetMStep(uint8_t mStep);
    
    private:
    uint8_t m_address = 0xE0;
    EventQueue *m_evQueue;
    UartCOM * m_uartCOM;// = UartCOM(PC_10, PC_11); // Each motor will have its own uart com.
};
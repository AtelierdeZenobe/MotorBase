/**
 * @brief for a 3-omniwheel robot only.
 *
 */

#include "mbed.h"
#include <cstdint>
#include <iostream>
#include <cmath>
#include <ostream>
#include "motor.h"
#include "robot.h"
#include "matrix.h"

int main()
{
    std::cout << "\n\nPrint from main" << std::endl;

    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    auto robot = new Robot(&EVqueue);
    //robot->InitializeMotorbase();
    //robot->Calibrate();
    //robot->SetACC(0x0100);
    //robot->SetACC(0x11e);
    //robot->SetPID(0x0650, 0x1, 0x250);

    double speed = 100;
    robot->Move(1000, 0, 0, speed, 0);
    wait_us(1000000);
    robot->Move(1000, 180, 0, speed, 0);
    // Should move for 10s
    //robot->Move(500, PI/2, 0, 100, 0);
    //robot->Move(500, 0, 0, 100, 0);
    //robot->Calibrate();

/*
    std::vector<int8_t> data = {0x00,0x60};
    auto uartCOM = new UartCOM(PC_10, PC_11);
    Message * messageOut = new Message(0xE1, SET_ACC, data);
    Message messageIn;
    uartCOM->Send(messageOut, messageIn);

*/

    EVqueue.dispatch_forever();    
};
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
    std::cout << "Print from main" << std::endl;

    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    auto robot = new Robot(&EVqueue);
    robot->InitializeMotorbase();

    //robot->SetACC(0x0100);
    robot->SetACC(0x11e);
    std::cout << "\n100 0 0 100" << std::endl;
    robot->Move(100, 0, 0, 100);
    std::cout << "\n100 90 0 100" << std::endl;
    robot->Move(100, 90, 0, 100);
    std::cout << "\n0 0 360 100" << std::endl;
    robot->SetPID(0x650, 0x1, 0x650);
    //robot->Move(0, 0, 360, 100);
    // Asking to move for 100mm at 10mm/s -> Should move for 10s
    robot->Move(100, 0, 0, 10);
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
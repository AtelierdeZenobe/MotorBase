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

#include "message.h"
#include "uartCOM.h"
#include "functionCodes.h"

int main()
{
    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    auto robot = new Robot(&EVqueue);
   // robot->InitializeMotorbase();
    //robot->CalibrateMotors();

    //robot->Move(0, 0, 720, 1, 8);

    robot->Move();


    std::vector<int8_t> data = {0x00};
    auto uartCOM = new UartCOM(PC_10, PC_11);
    Message * messageOut = new Message(0xE1, CALIBRATE, data);
    Message messageIn;
    uartCOM->Send(messageOut, messageIn);



    EVqueue.dispatch_forever();



    

};
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

#include "EasyCAT.h"

EasyCAT EASYCAT; 

int main()
{
    std::cout << "\n\nPrint from main" << std::endl;

    if(!EASYCAT.Init())
    {
        std::cout << "Easycat failed to init" << std::endl;
        printf("Init failed \n");
    }
    else
    {
        std::cout << "Easy cat initialized" << std::endl;
        printf("Init succesful \n");
    }

    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    auto robot = new Robot(&EVqueue);

    double speed = 100;
    robot->Move(1000, 0, 0, speed, 0);

    while(true)
    {
        uint8_t status = static_cast<uint8_t>(EASYCAT.MainTask() & 0x7f);
        if(status < 4)
        {
            std::cout << "Incorrect status: " << status << std::endl;
        }

        static int count = 0;
        if(count % 100 == 0)
        {
            printf("wanted distance MSB: %02x\n",EASYCAT.BufferOut.Byte[0]);
            printf("wanted distance: LSB: %02x\n",EASYCAT.BufferOut.Byte[1]);
            printf("wanted angle MSB: %02x\n",EASYCAT.BufferOut.Byte[2]);
            printf("wanted angle LSB: %02x\n",EASYCAT.BufferOut.Byte[3]);
        }

        EASYCAT.BufferIn.Byte[0]=0x01;
        EASYCAT.BufferIn.Byte[1]=0x02;
        EASYCAT.BufferIn.Byte[2]=0x03; 
    
        uint16_t wanted_speed = ((EASYCAT.BufferOut.Byte[0] << 8) & 0xF0) | EASYCAT.BufferOut.Byte[1];
        printf("wanted_speed %02x\n", wanted_speed);
        //ThisThread::sleep_for(100);
        if(EASYCAT.BufferOut.Byte[1] % 2 == 0)
        {
            //robot->Move(wanted_speed, 0, 0, 1, 32);
        }
        else
        {
            //robot->Move(wanted_speed, 180, 0, 1, 32);
        }
        
        EVqueue.dispatch_once();
        EVqueue.dispatch_once();
        EVqueue.dispatch_once();
    }
    EVqueue.dispatch_forever();    
};
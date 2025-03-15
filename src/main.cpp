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

        if(count++ % 1000 == 0 && false)
        {
            printf("wanted distance MSB: %02x\n",EASYCAT.BufferOut.Byte[0]);
            printf("wanted distance: LSB: %02x\n",EASYCAT.BufferOut.Byte[1]);
            printf("wanted angle MSB: %02x\n",EASYCAT.BufferOut.Byte[2]);
            printf("wanted angle LSB: %02x\n",EASYCAT.BufferOut.Byte[3]);
            printf("wanted rotation MSB: %02x\n",EASYCAT.BufferOut.Byte[4]);
            printf("wanted rotation: LSB: %02x\n",EASYCAT.BufferOut.Byte[5]);
            printf("wanted speed MSB: %02x\n",EASYCAT.BufferOut.Byte[6]);
            printf("wanted speed LSB: %02x\n",EASYCAT.BufferOut.Byte[7]);
        }

        EASYCAT.BufferIn.Byte[0]=0x01;
        EASYCAT.BufferIn.Byte[1]=0x02;
        EASYCAT.BufferIn.Byte[2]=0x03; 
    
        double wanted_distance = static_cast<double>(((EASYCAT.BufferOut.Byte[0] << 8) & 0xFF00) | EASYCAT.BufferOut.Byte[1]);
        double wanted_angle = static_cast<double>(((EASYCAT.BufferOut.Byte[2] << 8) & 0xFF00) | EASYCAT.BufferOut.Byte[3]);
        double wanted_rotation = static_cast<double>(((EASYCAT.BufferOut.Byte[4] << 8) & 0xFF00) | EASYCAT.BufferOut.Byte[5]);
        double wanted_speed = static_cast<double>(((EASYCAT.BufferOut.Byte[6] << 8) & 0xFF00) | EASYCAT.BufferOut.Byte[7]);
        if(count++ % 1001 == 0 && true)
        {
            printf("wanted_distance %f\n", wanted_distance);
            printf("wanted_angle %f\n", wanted_angle);
            printf("wanted_rotation %f\n", wanted_rotation);
            printf("wanted_speed %f\n", wanted_speed);
        }
        robot->Move(wanted_distance, wanted_angle, wanted_rotation, wanted_speed);
        
        EVqueue.dispatch_once();
        EVqueue.dispatch_once();
        EVqueue.dispatch_once();
    }
    EVqueue.dispatch_forever();    
};
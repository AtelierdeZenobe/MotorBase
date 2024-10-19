/**
 * @brief for a 3-omniwheel robot only.
 *
 */

#include <cstdint>
#include <iostream>

#include "mbed.h"
#include "EasyCAT.h"

#include "motor.h"
#include "robot.h"
#include "matrix.h"

EasyCAT EASYCAT; 

int main()
{

    ///
    /// Ethercat part
    /// The communication is critical. It must never hang for motor instructions.
    ///
    
    if(!EASYCAT.Init())
    {
        std::cout << "Easycat failed to init" << std::endl;
    }

    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));
    auto robot = new Robot(&EVqueue);
    robot->InitializeMotorbase();

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
        //TODO: EasyCAT.h offers a Custom Mode allowing so specify a data type

        // state is one byte and other is 2 bytes
        EASYCAT.BufferIn.Byte[0]=0x01;
        EASYCAT.BufferIn.Byte[1]=0x02;
        EASYCAT.BufferIn.Byte[2]=0x03; 

        // 
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
    
    //TODO: Handle ethercat using event loop

// Kinematics (p29) : https://pure.tue.nl/ws/portalfiles/portal/4274124/612987.pdf
// Kinematics (more explanations) : https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf

    //robot->CalibrateMotors();
    //robot->Move(0, 0, 720, 1, 8);

    robot->Move();

/*
    std::cout << *(robot->m_inverseJacobianKinematicsMatrix) << std::endl;
    std::cout << *(robot->m_wantedVelocityVector) << std::endl;
    std::cout << *(robot->m_wheelAngularSpeedVector) << std::endl;
*/

    EVqueue.dispatch_forever();
    
    while(true);
    // 1 revolution at lowest speed (Steps/revolution = 200)
    // Vrpm = (Speed x 30000)/(Mstep x 200) -> MAX 2000RPM
    // => At Mstep = 1, max speed is 
};
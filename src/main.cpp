/**
 * @brief for a 3-omniwheel robot only.
 *
 */

#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include <cmath> // For trigonometric functions
#include <ostream>
#include "motor.h"
#include "robot.h"
#include "matrix.h"

int main()
{
    EventQueue EVqueue;
    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

// Kinematics (p29) : https://pure.tue.nl/ws/portalfiles/portal/4274124/612987.pdf
// Kinematics (more explanations) : https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf

    auto robot = new Robot(&EVqueue);
    robot->InitializeMotorbase();
    //robot->CalibrateMotors();
    robot->Move(800, -60, 720, 1, 8);

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
#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include <cmath> //For trigonometric functions
#include "motor.h"
#include "robot.h"

#include "matrix-master/dist/matrix.h"
//TODO: Find why we can't print to the console
// For now: -> Switch USB

#define MSTEP 2
#define TPR 200 * MSTEP
#define PI 3.141592
#define DPR PI * 2 * B
#define DPT DPR / TPR




int main()
{
    EventQueue EVqueue;

    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    // Dummy showing that it will not block
    //Motor dummy_motor(0x69, &EVqueue); // No uart mapped to 0x69 -> Will wait for an answer forever
    //dummy_motor.Go(1, 1, 200);

// Kinematics (p29) : https://pure.tue.nl/ws/portalfiles/portal/4274124/612987.pdf

    const uint8_t motors = 3; //MAX 9
    auto robot = new Robot<motors>(&EVqueue);
    robot->setMotors();
    robot->setMatrix(116.0f, 30.0f, 60.0f, 0.0f, 120.0f, 240.0f);
    robot->setVelocity(100.0f, 0.0f, 0.0f)->findPhi();


    //std::cout << *(robot->getPhi()) << std::endl;

    //auto phi = robot->setVelocity(100.0f, 0.0f, 0.0f)->getPhi();

    std::cout << robot->absPhi(2) << std::endl;

    //robot->getMotor(Robot<motors>::Motors::Motor3)
        //->Go(static_cast<uint8_t>(Robot<motors>::Direction::CounterClockWise),robot->absPhi(2),200);
    
    



    /*Motor* motors[3] = { new Motor(0xE1, &EVqueue), new Motor(0xE2, &EVqueue), new Motor(0xE3, &EVqueue) };

    robot->setMotor(new Motor(0xE1, &EVqueue));
    robot->setMotor(new Motor(0xE2, &EVqueue));
    robot->setMotor(new Motor(0xE3, &EVqueue));*/
    

    //(*phi)(0,0) > 0 ? motors[MOTOR_1]->Go(0, (*phi)(0,0), 200/*v(0,0)/DPT*/) : motors[MOTOR_1]->Go(1, -(*phi)(0,0), 200/*v(0,0)/DPT*/) ;
    //(*phi)(1,0) > 0 ? motors[MOTOR_2]->Go(0, (*phi)(1,0), 200/*v(1,0)/DPT*/) : motors[MOTOR_2]->Go(1, -(*phi)(1,0), 200/*v(1,0)/DPT*/) ;
    //(*phi)(2,0) > 0 ? motors[MOTOR_3]->Go(0, (*phi)(2,0), 200/*v(2,0)/DPT*/) : motors[MOTOR_3]->Go(1, -(*phi)(2,0), 200/*v(2,0)/DPT*/) ;
    
    //motors[MOTOR_1]->Go(static_cast<uint8_t>(Robot<wheels>::Direction::Clockwise),1,200);
    //motors[MOTOR_2]->Go(1,1,200);
    //motors[MOTOR_3]->Go(1,1,200);

    //robot->Move(100.0f, 0.0f, 0.0f); //Moves 100 along x-axis 
    //robot->Move(0.0f, 0.0f, 100.0f); //Turns 100

    

    printMutex.lock();

    //std::cout << *(robot->getMatrix()) << std::endl << *(robot->getVelocity()) << std::endl << *(robot->getPhi()) << std::endl;
    //std::cout << *(robot->getPhi()) << std::endl;
    //printf("")
    printMutex.unlock();
   // motors[MOTOR_2]
    //motors[MOTOR_3]
    

    /*for(auto i = 0; i < 3; i++)
    {
        motors[i]->Go(1, 1, 200*m(0,i));
    }*/
    //motors[MOTOR_1]->Go(1,1,200);
    //motors[MOTOR_2]->Go(0,1,200);
    
    //Motor motor(0xE2, &EVqueue);
    //motor.Go(1, 1, 200);
    
    /* Works
    auto lambda = [&motor]() {
        motor.Go(1,1,200);
    };
    EVqueue.call(lambda);
    EVqueue.dispatch_forever();
    */

    EVqueue.dispatch_forever();
    while(true);
    // 1 revolution at lowest speed (Steps/revolution = 200)
    // Vrpm = (Speed x 30000)/(Mstep x 200) -> MAX 2000RPM
    // => At Mstep = 1, max speed is 
    
    //motor.Go(1, 1, 200);

};
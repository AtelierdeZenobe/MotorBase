#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include <cmath> //For trigonometric functions
#include "motor.h"
#include "robot.h"

#include "matrix-master/dist/matrix.h"
//TODO: Find why we can't print to the console
// For now: -> Switch USB



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
    auto robot = new Robot(&EVqueue, motors);
    robot->Initialize(120.0f, 30.0f, 60.0f);

    //robot->Calibrate();

    int dest_x = 0;
    int dest_y = 0;
    int v_x = 100;
    int v_y = 100;
    double omega = 0;


    robot->Move(dest_x, dest_y, v_x, v_y, omega); //when velocity was modified
    //robot->Move(dest_x, dest_y); //with same velocity
    //robot->Move(dest_x, dest_y, t); //

    


    ///////////
    //FINAL
    ///////////
    /*
    auto robot = new Robot(...);
    robot->Initialize();
    Robot->WhereAmI() | robot->Pos(); 
    robot->Move(x, y, theta) | robot->Move(rho, theta);
    */


    


    //robot()->Move(100.0f, 0.0f, 0.0f); //Moves 100 along x-axis 
    //robot->Move(0.0f, 0.0f, 100.0f); //Turns 100


    
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
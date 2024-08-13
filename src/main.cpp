#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include "motor.h"

#include "matrix-master/dist/matrix.h"
//TODO: Find why we can't print to the console
// For now: -> Switch USB

#define SIN60 .866025
#define SIN180 -1.000000
#define SIN300 -.866025
#define COS60 .500000
#define COS180 0.000000
#define COS300 -.500000
#define R 116
#define B 30
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
    Matrix m(3,3);
    m(0,0) = -SIN60;
    m(0,1) = COS60;
    m(0,2) = R;
    m(1,0) = -SIN180;
    m(1,1) = COS180;
    m(1,2) = R;
    m(2,0) = -SIN300;
    m(2,1) = COS300;
    m(2,2) = R;
    m = (1.0/B) * m;


    float Vx = 0;
    float Vy = 100;
    float w = 100;

    Matrix v(3,1);
    v(0,0) = Vx;
    v(1,0) = Vy;
    v(2,0) = w;
    
    Matrix phi(3,1);

    
    phi = m * v;

    Motor* motors[3] = { new Motor(0xE1, &EVqueue), new Motor(0xE2, &EVqueue), new Motor(0xE3, &EVqueue) };
    phi(0,0) > 0 ? motors[MOTOR_1]->Go(1, phi(0,0), 200/*v(0,0)/DPT*/) : motors[MOTOR_1]->Go(0, -phi(0,0), 200/*v(0,0)/DPT*/) ;
    phi(1,0) > 0 ? motors[MOTOR_2]->Go(1, phi(1,0), 200/*v(1,0)/DPT*/) : motors[MOTOR_2]->Go(0, -phi(1,0), 200/*v(1,0)/DPT*/) ;
    phi(2,0) > 0 ? motors[MOTOR_3]->Go(1, phi(2,0), 200/*v(2,0)/DPT*/) : motors[MOTOR_3]->Go(0, -phi(2,0), 200/*v(2,0)/DPT*/) ;
    
    //motors[MOTOR_1]->Go(1,1,200);

double z = 0.2;
    printMutex.lock();
    for(int i = 0; i < 3; i++){
        printf("%lf\n", phi(i,0));
    }

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
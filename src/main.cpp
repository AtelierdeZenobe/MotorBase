#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include "motor.h"
//TODO: Find why we can't print to the console
// For now: -> Switch USB


int main()
{
    EventQueue EVqueue;

    Thread eventThread;
    eventThread.start(callback(&EVqueue, &EventQueue::dispatch_forever));

    // Dummy showing that it will not block
    Motor dummy_motor(0x69, &EVqueue); // No uart mapped to 0x69 -> Will wait for an answer forever
    dummy_motor.Go(1, 1, 200);
    Motor motor(0xE1, &EVqueue);
    motor.Go(1, 1, 200);
    
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
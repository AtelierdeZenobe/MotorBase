#include "mbed.h"
#include <cstdint> // For uint8_t
#include <iostream>
#include "motor.h"
//TODO: Find why we can't print to the console
// For now: -> Switch USB
int main()
{
    Motor motor = Motor(0xE0, Position());
    // 1 revolution at lowest speed (Steps/revolution = 200)
    motor.Go(1, 1, 200);
}
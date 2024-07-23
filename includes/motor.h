#include <cstdint> // For uint8_t
#include "mbed.h"
#include "uartCOM.h"
struct Position
{
    uint8_t x = 0x00;
    uint8_t y = 0x00;
    uint8_t t = 0x00;
};

/**
* @brief: Represents a single motor.
* 
* The motor is able to move in a certain direction and speed, for a number of steps.
* It is possible to configure some motor parameters like zero position etc.
*/
class Motor
{
    public:
    Motor(uint8_t address, Position position);
    bool Go(uint8_t direction, uint8_t speed, uint32_t nbSteps);
    bool SetZero();
    bool GoToZero();

    private:
    uint8_t m_address = 0xE0;
    UartCOM m_uartCOM = UartCOM(); // Each motor will have its own uart com.
    Position m_position = {0x00, 0x00, 0x00}; // TODO: put the initial position
};
#include"motor.h"
#include<iostream>
//#include "message.h"
#include "functionCodes.h"
Motor::Motor(uint8_t address, Position position) : m_address(address), m_position(position)
{
    printf("Initialised motor %02x at position (%02x,%02x,%02x)\n", m_address, m_position.x, m_position.y, m_position.t);
}

bool Motor::Go(uint8_t direction, uint8_t speed, uint32_t nbSteps)
{
    bool success = true;
    // Check arguments

    if(direction>1)
    {
        success = false;
        printf("Motor::Go| direction is greater than 1 (%02x).\n", direction);
    }
    else if (nbSteps < 0)
    {
        success = false;
        printf("Motor::Go| nbSteps is less than 0 (%02x).\n", nbSteps);
    }

    //TODO: check for max RPM
    
    //Construct message
    if(success)
    {
        std::vector<uint8_t> data;
        data.push_back( (direction << 7) | (speed &= 0x7F)); // 1st bit is direction, others are speed
        data.push_back(static_cast<uint8_t>((nbSteps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(nbSteps & 0xFF));
        Message messageOut = Message(m_address, RUN_DIR_PULSES, data);
        messageOut.display();
        Message messageIn;
        m_uartCOM.Send(messageOut, messageIn);
    }

    return success;
}

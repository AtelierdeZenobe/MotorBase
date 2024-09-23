#include"motor.h"
#include <cstdint>
#include<iostream>
#include <vector>
//#include "message.h"
#include "functionCodes.h"
#include "uartmap.h"

Motor::Motor(uint8_t address, EventQueue* evQueue) : m_address(address), m_evQueue(evQueue)
{
    auto it = UART_MAP.find(m_address);
    printMutex.lock();
    //printf("Initializing motor's uart.\n");
    printMutex.unlock();
    if(it != UART_MAP.end())
    {
        auto pins = it->second;
        m_uartCOM = new UartCOM(pins.first, pins.second);
    }
    else
    {
        // Should start looking for uart with a send receive logic
        // For now, UART 6
        printMutex.lock();
        //printf("%02x| Could not find address in uart map.\n", m_address);
        printMutex.unlock();
        m_uartCOM = new UartCOM(PC_6, PC_7);
    }
    printMutex.lock();
    //printf("Initialised motor %02x.\n", m_address);
    printMutex.unlock();
}

Motor::~Motor()
{
    delete(m_uartCOM);
    delete(m_evQueue);
}

void Motor::Go(int8_t dirspeed, uint32_t nbSteps)
{

    bool success = true;

    if (nbSteps < 0)
    {
        success = false;
        printMutex.lock();
        //printf("Motor::Go| nbSteps is less than 0 (%02x).\n", nbSteps);
        printMutex.unlock();
    }

    //TODO: check for max RPM
    
    //Construct message
    if(success)
    {

        std::cout << std::bitset<8>(dirspeed) << " -> ";
        if(dirspeed < 0)
        {
            dirspeed = -dirspeed;
            dirspeed |= 1 << 7;
        }
        std::cout << std::bitset<8>(dirspeed) << std::endl;

        std::vector<int8_t> data;
        //data.push_back( (direction << 7) | (speed &= 0x7F)); // 1st bit is direction, others are speed
        data.push_back(static_cast<int8_t>(dirspeed & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(nbSteps & 0xFF));
        Message* messageOut = new Message(m_address, RUN_DIR_PULSES, data);
        messageOut->display();
        Message messageIn;
        
        // TODO: clarify
        auto lambda = [this, &messageIn, messageOut]()
        {
            m_uartCOM->Send(messageOut, messageIn);
        };
        m_evQueue->call(lambda);
        //m_evQueue->dispatch_once();
        //m_uartCOM->Send(messageOut, messageIn);
    }

    //return success;
}

void Motor::Go(uint8_t direction, uint8_t speed, uint32_t nbSteps)
{

    bool success = true;

    // Check arguments
    if(direction>1)
    {
        success = false;
        printMutex.lock();
        //printf("Motor::Go| direction is greater than 1 (%02x).\n", direction);
        printMutex.unlock();
    }
    else if (nbSteps < 0)
    {
        success = false;
        printMutex.lock();
        //printf("Motor::Go| nbSteps is less than 0 (%02x).\n", nbSteps);
        printMutex.unlock();
    }

    //TODO: check for max RPM
    
    //Construct message
    if(success)
    {
        std::vector<int8_t> data;
        data.push_back( (direction << 7) | (speed &= 0x7F)); // 1st bit is direction, others are speed
        data.push_back(static_cast<int8_t>((nbSteps >> 24) & 0xFF));
        data.push_back(static_cast<int8_t>((nbSteps >> 16) & 0xFF));
        data.push_back(static_cast<int8_t>((nbSteps >> 8) & 0xFF));
        data.push_back(static_cast<int8_t>(nbSteps & 0xFF));
        Message* messageOut = new Message(m_address, RUN_DIR_PULSES, data);
        messageOut->display();
        Message messageIn;
        
        // TODO: clarify
        auto lambda = [this, &messageIn, messageOut]()
        {
            m_uartCOM->Send(messageOut, messageIn);
        };
        m_evQueue->call(lambda);
        //m_evQueue->dispatch_once();
        //m_uartCOM->Send(messageOut, messageIn);
    }

    //return success;
}
/*
bool Motor::Calibrate()
{
    Message * messageOut = new Message(m_address, CALIBRATE, std::vector<int8_t>());
    Message messageIn;
    m_uartCOM->Send(messageOut, messageIn);
    return true;
}
*/
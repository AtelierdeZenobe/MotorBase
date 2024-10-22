#include"motor.h"
#include <cstdint>
#include<iostream>
#include <memory>
#include <vector>
//#include "message.h"
#include "functionCodes.h"
#include "uartmap.h"

Motor::Motor(uint8_t address, EventQueue* evQueue) : m_address(address), m_evQueue(evQueue)
{
    auto it = UART_MAP.find(m_address);

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
        printf("%02x| Could not find address in uart map.\n", m_address);
        printMutex.unlock();
        m_uartCOM = new UartCOM(PC_6, PC_7);
    }
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
        printf("Motor::Go| nbSteps is less than 0 (%02x).\n", nbSteps);
        printMutex.unlock();
    }

    //TODO: check for max RPM
    
    //Construct message
    if(success)
    {

        //std::cout << std::bitset<8>(dirspeed) << " -> ";
        if(dirspeed < 0)
        {
            dirspeed = -dirspeed;
            dirspeed |= 1 << 7;
        }
        //std::cout << std::bitset<8>(dirspeed) << std::endl;

        std::vector<uint8_t> data;
        //data.push_back( (direction << 7) | (speed &= 0x7F)); // 1st bit is direction, others are speed
        data.push_back(static_cast<uint8_t>(dirspeed & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(nbSteps & 0xFF));
        Message* messageOut = new Message(m_address, RUN_DIR_PULSES, data);
        //messageOut->display();
        auto messageIn = std::make_shared<Message>();
        
        // TODO: clarify
        auto lambda = [this, &messageIn, messageOut]()
        {
            m_uartCOM->Send(messageOut, messageIn);
            if(m_uartCOM->getState() != uartSM::UART_DONE)
            {
                std::cout << "uart not not done" << std::endl;
            }
            else
            {
                //messageIn->display();
            }
        };
        m_evQueue->call(lambda);

/*
        auto lambda2 = [this, messageIn, messageOut]()
        {
            uartSM state = m_uartCOM->getState();
            if(state != uartSM::UART_DONE)
            {
                std::cout << "Not ready: " << static_cast<uint8_t>(state) << std::endl;
            }
            messageIn->display();
        };
        m_evQueue->call(lambda2);
*/
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
        std::vector<uint8_t> data;
        data.push_back( (direction << 7) | (speed &= 0x7F)); // 1st bit is direction, others are speed
        data.push_back(static_cast<uint8_t>((nbSteps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((nbSteps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(nbSteps & 0xFF));
        Message* messageOut = new Message(m_address, RUN_DIR_PULSES, data);
        //messageOut->display();
        Message messageIn;
        
        // TODO: clarify
        auto lambda = [this, &messageIn, messageOut]()
        {
            //m_uartCOM->Send(messageOut, messageIn);
        };
        m_evQueue->call(lambda);
        //m_evQueue->dispatch_once();
        //m_uartCOM->Send(messageOut, messageIn);
    }

    //return success;
}

bool Motor::Calibrate()
{
    std::vector<uint8_t> data = {0x00};
    Message * messageOut = new Message(m_address, CALIBRATE, data);
    Message messageIn;
    //m_uartCOM->Send(messageOut, messageIn);
    //messageOut->display();
    return true;
}

bool Motor::SetPID(uint16_t kp, uint16_t ki, uint16_t kd)
{
    std::vector<uint8_t> data = {(uint8_t)((kp >> 8) & 0xFF), (uint8_t)(kp & 0xFF)};
    Message * messageOut = new Message(m_address, SET_KP_POS, data);
    Message messageIn;
    //m_uartCOM->Send(messageOut, messageIn);


    data = {(uint8_t)((ki >> 8) & 0xFF), (uint8_t)(ki & 0xFF)};
    messageOut = new Message(m_address, SET_KI_POS, data);
    //m_uartCOM->Send(messageOut, messageIn);


    data = {(uint8_t)((kd >> 8) & 0xFF), (uint8_t)(kd & 0xFF)};
    messageOut = new Message(m_address, SET_KD_POS, data);
    //m_uartCOM->Send(messageOut, messageIn);


    return true;

}


bool Motor::SetACC(uint16_t ACC)
{

    std::vector<uint8_t> data = {(uint8_t)((ACC >> 8) & 0xFF), (uint8_t)(ACC & 0xFF)};
    Message * messageOut = new Message(m_address, SET_ACC, data);
    Message messageIn;
    //m_uartCOM->Send(messageOut, messageIn);

    return true;

}

bool Motor::SetMStep(uint8_t mStep)
{

    std::vector<uint8_t> data = {(uint8_t)mStep};
    Message * messageOut = new Message(m_address, SET_SUBDIVISION, data);
    Message messageIn;
    //m_uartCOM->Send(messageOut, messageIn);

    return true;

}


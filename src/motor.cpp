#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
#include <bitset>

#include "motor.h"
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

void Motor::Go(uint8_t dirspeed, uint32_t nbSteps)
{
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(dirspeed & 0xFF));
    data.push_back(static_cast<uint8_t>((nbSteps >> 24) & 0xFF));
    data.push_back(static_cast<uint8_t>((nbSteps >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((nbSteps >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(nbSteps & 0xFF));
    std::shared_ptr<MessageOut> messageOut = std::make_shared<MessageOut>(m_address, RUN_DIR_PULSES, data);
    
    auto lambda = [this, messageOut]()
    {
        auto answer = m_uartCOM->Send(messageOut);
        answer.display();

    };
    m_evQueue->call(lambda);
}

/*
bool Motor::Calibrate()
{
    MessageIn messageIn();
    messageIn = m_uartCOM->Send(std::make_shared<MessageOut>(m_address, CALIBRATE));
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
*/
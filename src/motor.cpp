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
        // const auto answer =
        m_uartCOM->Send(messageOut);
        //answer.display();
    };
    m_evQueue->call(lambda);
}

// TODO: command function
bool Motor::Calibrate()
{
    // TODO: Simultaneous.
    bool success = true;
    std::shared_ptr<MessageOut> command = std::make_shared<MessageOut>(m_address, CALIBRATE, std::vector<uint8_t>{0x00});
    m_uartCOM->Send(command);
    // const auto messageIn = m_uartCOM->Send(command);
    // if(messageIn.getData()[0] != 0x01)
    // {
    //     printMutex.lock();
    //     printf("Calibrate returned %d instead of 0x01.\n", messageIn.getData()[0]);
    //     printMutex.unlock();
    //     success = false;
    // }
    // return success;
}


bool Motor::SetPID(uint16_t kp, uint16_t ki, uint16_t kd)
{
    bool success = true;

    // Loop over the commands, get the result
    std::vector<uint16_t> factors = {kp, ki, kd};
    std::vector<uint8_t> commands = {SET_KP_POS, SET_KI_POS, SET_KD_POS};

    for(size_t i = 0; i < commands.size(); ++i)
    {
        std::vector<uint8_t> data = {(uint8_t)((factors[i] >> 8) & 0xFF), (uint8_t)(factors[i] & 0xFF)};
        std::shared_ptr<MessageOut> command = std::make_shared<MessageOut>(m_address, commands[i], data);
        m_uartCOM->Send(command);
        // auto messageIn = m_uartCOM->Send(command);

        // if(messageIn.getData()[0] != 0x01)
        // {
        //     printMutex.lock();
        //     printf("SetPID returned %d instead of 0x01.\n", messageIn.getData()[0]);
        //     printMutex.unlock();
        //     success = false;
        // }
    }

    return success;
}


bool Motor::SetACC(uint16_t ACC)
{
    bool success = true;

    std::vector<uint8_t> data = {(uint8_t)((ACC >> 8) & 0xFF), (uint8_t)(ACC & 0xFF)};
    std::shared_ptr<MessageOut> command = std::make_shared<MessageOut>(m_address, SET_ACC, data);
    m_uartCOM->Send(command);
    // const auto messageIn = m_uartCOM->Send(command);
    /*
    messageIn.display();
    if(messageIn.getData()[0] != 0x01)
    {
        printMutex.lock();
        printf("SetACC returned %d instead of 0x01.\n", messageIn.getData()[0]);
        printMutex.unlock();
        success = false;
    }
    */
    return success;
}

bool Motor::SetMStep(uint8_t mStep)
{
    bool success = true;
    std::vector<uint8_t> data = {(uint8_t)mStep};
    std::shared_ptr<MessageOut> command = std::make_shared<MessageOut>(m_address, SET_SUBDIVISION, data);
    m_uartCOM->Send(command);
    // const auto messageIn = m_uartCOM->Send(command);
    // if(messageIn.getData()[0] != 0x01)
    // {
    //     printMutex.lock();
    //     printf("SetMStep returned %d instead of 0x01.\n", messageIn.getData()[0]);
    //     printMutex.unlock();
    //     success = false;
    // }

    return success;
}

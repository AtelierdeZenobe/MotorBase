#include "robot.h"
#include <cstdint>

template <>
Robot<tMotors>::Robot(EventQueue* EVqueue) : m_EVqueue(EVqueue)
{
    if(tMotors > maxWheels)
    {
        return;
    }

    m_velocity = new Matrix(3, 1);
    m_matrix = new Matrix(tMotors, 3);
    m_phi = new Matrix(tMotors, 1);
}

template <>
void Robot<tMotors>::setMotors()
{
    uint8_t address = m_firstAddress;

    for(uint8_t i = 0; i < tMotors; i++)
    {
        m_motors[i] = new Motor(address + i, m_EVqueue);
    }
}

template <>
Motor* Robot<tMotors>::getMotor(Robot<tMotors>::Motors motor)
{
    if(static_cast<uint8_t>(motor) >= tMotors)
    {
        return nullptr;
    }

    return m_motors[static_cast<uint8_t>(motor)];
}

template <>
void Robot<tMotors>::setMatrix(float r, float b, float theta, float alpha_1, float alpha_2, float alpha_3)
{
    if(r <= 0 || b <= 0)
    {
        return;
    }

    (*m_matrix)(0,0) = -sin(m_DEG2RAD * (theta + alpha_1));
    (*m_matrix)(0,1) = cos(m_DEG2RAD * (theta + alpha_1));
    (*m_matrix)(0,2) = r;
    (*m_matrix)(1,0) = -sin(m_DEG2RAD * (theta + alpha_2));
    (*m_matrix)(1,1) = cos(m_DEG2RAD * (theta + alpha_2));
    (*m_matrix)(1,2) = r;
    (*m_matrix)(2,0) = -sin(m_DEG2RAD * (theta + alpha_3));
    (*m_matrix)(2,1) = cos(m_DEG2RAD * (theta + alpha_3));
    (*m_matrix)(2,2) = r;

    (*m_matrix) = (1.0f / b) * (*m_matrix);
}

template <>
Robot<tMotors>* Robot<tMotors>::setVelocity(float v_x, float v_y, float omega)
{
    (*m_velocity)(0,0) = v_x;
    (*m_velocity)(1,0) = v_y;
    (*m_velocity)(2,0) = omega;

    return this;
}

template <>
Matrix* Robot<tMotors>::findPhi(void)
{
    *m_phi = (*m_matrix) * (*m_velocity);

    return m_phi;
}

template <>
Robot<tMotors>::Direction Robot<tMotors>::signPhi(int row)
{
    if((*m_phi)(row, 0) > 0)
    {
        return Robot<tMotors>::Direction::CounterClockWise;
    }
    
    return Robot<tMotors>::Direction::ClockWise;
}

template <>
uint8_t Robot<tMotors>::absPhi(int row)
{
    float phi = (*m_phi)(row, 0);
    if(phi < 0)
    {
        phi = -phi;
    }


    // DOES NOT WORK YET
    //std:: cout << phi << std::endl;

    //CONVERT FLOAT TO UINT8_T
    //uint8_t* pPhi = reinterpret_cast<uint8_t*>(&phi);

    //std::cout << "hello : " << pPhi[0] << std::endl;

    //return *pPhi;

   // return static_cast<uint8_t>(phi)';
   return 1;

}


template <>
Matrix* Robot<tMotors>::getPhi(void)
{
    return m_phi;
}

/*

Matrix* Robot::getMatrix(void)
{
    return m_matrix;
}

Matrix* Robot::getVelocity(void)
{
    return m_velocity;
}

void Robot::Move(float rho, float theta)
{
    //Move(rho * cos(theta), rho * sin(theta), ??)
}

void Robot::Move(float x, float y, float omega)
{
    //this->setVelocity(x, y, omega);
}



void Robot::Pos()
{

}
*/
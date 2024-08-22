#include "robot.h"
#include <cstdint>

Robot::Robot(EventQueue* EVqueue, uint8_t motor, uint8_t firstMotorAddress) 
    : m_EVqueue(EVqueue), m_motor(motor), m_firstMotorAddress(firstMotorAddress)
{
    if(m_motor > Robot::MaxMotors)
    {
        return; //Legit? Exception? Who cares?
    }

    //Creates all kinematics matrixes needed to move on
    m_velocity = new Matrix(3, 1);
    m_matrix = new Matrix(m_motor, 3);
    m_phi = new Matrix(m_motor, 1);

    // Creates all motors with a specific automatically given address
    initMotors();
}

void Robot::Initialize(float r, float b, float theta, int pos_x, int pos_y, float alpha_1, float alpha_2, float alpha_3)
{
    initPos(pos_x, pos_y);
    initMatrix(r, b, theta, alpha_1, alpha_2, alpha_3);
}

void Robot::initMotors(void)
{
    uint8_t address = m_firstMotorAddress;

    for(uint8_t i = 0; i < m_motor; i++)
    {
        m_motors[i] = new Motor(address + i, m_EVqueue);
    }
}

void Robot::initMatrix(float r, float b, float theta, float alpha_1, float alpha_2, float alpha_3)
{
    if(r <= 0 || b <= 0)
    {
        return;
    }

    (*m_matrix)(0,0) = -sin(Robot::DegToRad * (theta + alpha_1));
    (*m_matrix)(0,1) = cos(Robot::DegToRad * (theta + alpha_1));
    (*m_matrix)(0,2) = r;
    (*m_matrix)(1,0) = -sin(Robot::DegToRad * (theta + alpha_2));
    (*m_matrix)(1,1) = cos(Robot::DegToRad * (theta + alpha_2));
    (*m_matrix)(1,2) = r;
    (*m_matrix)(2,0) = -sin(Robot::DegToRad * (theta + alpha_3));
    (*m_matrix)(2,1) = cos(Robot::DegToRad * (theta + alpha_3));
    (*m_matrix)(2,2) = r;

    (*m_matrix) = (1.0f / b) * (*m_matrix);
}

void Robot::initPos(int pos_x, int pos_y)
{
    m_currentPos->x = pos_x;
    m_currentPos->y = pos_y;
}

struct Robot::Pos* Robot::Pos(void)
{
    return m_currentPos;
}

bool Robot::Calibrate(void)
{
    for(uint8_t i = 0; i < m_motor; i++)
    {
        if(m_motors[i]->Calibrate() == false)
        {
            return false;
        }
    }

    return true;
}



//----------------------------





Robot* Robot::setVelocity(float v_x, float v_y, float omega)
{
    (*m_velocity)(0,0) = v_x;
    (*m_velocity)(1,0) = v_y;
    (*m_velocity)(2,0) = omega;

    return this;
}

Matrix* Robot::findPhi(void)
{
    *m_phi = (*m_matrix) * (*m_velocity);

    return m_phi;
}

uint8_t Robot::signPhi(int row)
{
    auto direction = static_cast<uint8_t>(Robot::Direction::ClockWise);

    if((*m_phi)(row, 0) > 0)
    {
        direction = static_cast<uint8_t>(Robot::Direction::CounterClockWise);
    }
    
    return direction;
}

unsigned int Robot::absPhi(int row)
{
    float phi = (*m_phi)(row, 0);
    if(phi < 0)
    {
        phi = -phi;
    }

   return static_cast<unsigned int>(phi);
}

Matrix* Robot::getPhi(void)
{
    return m_phi;
}


bool Robot::Move(int dest_x, int dest_y, int v_x, int v_y, int omega)
{
    setVelocity(v_x, v_y, omega);
    findPhi();

    for(uint8_t i = 0; i < m_motor; i++)
    {
        //m_motors[i]->Go(signPhi(i),absPhi(i),..);
    }

    return true;
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

*/
#include "robot.h"

Robot::Robot(EventQueue* EVqueue) : m_EVqueue(EVqueue)
{

}

Robot::~Robot()
{
    for(int i = 0; i < N_MOTOR; i++)
    {
        delete m_motors[i]; //delete[] ?
    }

    delete m_wheelAngularSpeedVector;
    delete m_inverseJacobianKinematicsMatrix;
    delete m_wantedVelocityVector;
}


bool Robot::InitializeMotorbase()
{
// CHECK VALUES
    if(ROBOT_RADIUS <= 0 || WHEEL_RADIUS <= 0 || X_AXIS_ANGLE_MOTORS.size() != N_MOTOR || ADDRESS_MOTORS.size() != N_MOTOR)
    {
        return false;
    }

// INIT MOTORS
    for(int i = 0; i < N_MOTOR; i++)
    {
        m_motors[i] = new Motor(ADDRESS_MOTORS[i], m_EVqueue);

        if(m_motors[i] == nullptr)
        {
            return false;
        }
    }

// INIT KINEMATICS - MATRIXES
    m_wheelAngularSpeedVector = new Matrix(N_MOTOR, 1);
    m_inverseJacobianKinematicsMatrix = new Matrix(N_MOTOR, 3);
    m_wantedVelocityVector = new Matrix(3, 1);

    if(m_wheelAngularSpeedVector == nullptr || m_inverseJacobianKinematicsMatrix == nullptr || m_wantedVelocityVector == nullptr)
    {
        return false;
    }

// COMPUTE KINEMATICS - INVERSE JACOBIAN MATRIX 
    for(int i = 0; i < N_MOTOR; i++)
    {
        (*m_inverseJacobianKinematicsMatrix)(i,0) = -sin(DEG_TO_RAD * X_AXIS_ANGLE_MOTORS[i]);
        (*m_inverseJacobianKinematicsMatrix)(i,1) = cos(DEG_TO_RAD * X_AXIS_ANGLE_MOTORS[i]);
        (*m_inverseJacobianKinematicsMatrix)(i,2) = ROBOT_RADIUS;   
    }

    (*m_inverseJacobianKinematicsMatrix) *= (1.0 / WHEEL_RADIUS);

    return true;
}

bool Robot::Move(const int& wanted_distance, const int& wanted_angle, const int& wanted_rotation, 
    int wanted_speed, const int& wanted_mstep)
{
//CHECK VALUES
    if(wanted_speed < RPM_MINIMUM)
    {
        wanted_speed = RPM_MINIMUM;
    }
    else if(wanted_speed > RPM_MAXIMUM)
    {
        wanted_speed = RPM_MAXIMUM;
    }

    //Check that wanted_mstep is in a correct range and that it is a power of 2 also
    if(wanted_mstep < MSTEP_MINIMUM || wanted_mstep > MSTEP_MAXIMUM || (wanted_mstep & (wanted_mstep - 1)) != 0)
    {
        return false;
    }

// INIT KINEMATICS - WANTED VELOCITY VECTOR
    (*m_wantedVelocityVector)(0,0) = wanted_distance * cos(DEG_TO_RAD * wanted_angle);
    (*m_wantedVelocityVector)(1,0) = wanted_distance * sin(DEG_TO_RAD * wanted_angle);
    (*m_wantedVelocityVector)(2,0) = DEG_TO_RAD * wanted_rotation;

// COMPUTE KINEMATICS
    (*m_wheelAngularSpeedVector) = (*m_inverseJacobianKinematicsMatrix) * (*m_wantedVelocityVector);

    double steps_factor = wanted_mstep * TICS_PER_ROTATION / (2 * PI);
    double speed_factor = wanted_mstep * wanted_speed * TICS_PER_ROTATION * WHEEL_RADIUS * 1.0 
        / (SPEED_TO_RPM_FACTOR * SPEED_CORRECTION_FACTOR * Matrix::normVector(*m_wheelAngularSpeedVector));

// SEND DATA TO MOTORS
    for(int i = 0; i < N_MOTOR; i++)
    {
        m_motors[i]->Go(static_cast<int>((*m_wheelAngularSpeedVector)(i,0) * speed_factor), 
            static_cast<int>(std::abs((*m_wheelAngularSpeedVector)(i,0)) * steps_factor));
    }

    return true;
}
/*
bool Robot::CalibrateMotors(void)
{
    
    for(int i = 0; i < N_MOTOR; i++)
    {
        m_motors[i]->Calibrate();
    }

    return true;
}
*/
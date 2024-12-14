#include "robot.h"
#include <istream>

// Kinematics (p29) : https://pure.tue.nl/ws/portalfiles/portal/4274124/612987.pdf
// Kinematics (more explanations) : https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf


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

bool Robot::InitializeMotorbase(void)
{
// CHECK VALUES
    if(ROBOT_RADIUS <= 0 || WHEEL_RADIUS <= 0 || N_MOTOR != 3 || X_AXIS_ANGLE_MOTORS.size() != N_MOTOR || ADDRESS_MOTORS.size() != N_MOTOR)
    {
        std::cout << "Error: Initialization check values failed" << std::endl;
        return false;
    }

// INIT MOTORS
    for(int i = 0; i < N_MOTOR; i++)
    {
        m_motors[i] = new Motor(ADDRESS_MOTORS[i], m_EVqueue);

        if(m_motors[i] == nullptr)
        {
            std::cout << "Error: Unable to create the motor" << std::endl;
            return false;
        }
    }

    //Set mStep
    SetMStep(static_cast<uint8_t>(MSTEP)); //COnversion is OK cos 0 <= MSTEP <256


// INIT KINEMATICS - MATRIXES
    m_wheelAngularSpeedVector = new Matrix(N_MOTOR, 1);
    m_inverseJacobianKinematicsMatrix = new Matrix(N_MOTOR, 3);
    m_wantedVelocityVector = new Matrix(3, 1);

    if(m_wheelAngularSpeedVector == nullptr || m_inverseJacobianKinematicsMatrix == nullptr || m_wantedVelocityVector == nullptr)
    {
        std::cout << "Error: Unable to create kinematics matrixes" << std::endl;
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
    const int& wanted_speed, const int& wanted_mstep)
{
//CHECK VALUES
    if(N_MOTOR != 3) //N_MOTOR
    {
        std::cout << "Error: Invalid N_MOTOR value found" << std::endl;
        return false;
    }

    if(wanted_speed < RPM_MINIMUM || wanted_speed > RPM_MAXIMUM)
    {
        std::cout << "Error: wanted_speed is not in the range" << std::endl;
        return false;
    }

    //Check that wanted_mstep is in a correct range and that it is a power of 2 also
    if(wanted_mstep < MSTEP_MINIMUM || wanted_mstep > MSTEP_MAXIMUM || (wanted_mstep & (wanted_mstep - 1)) != 0)
    {
        std::cout << "Error: wanted_mstep is not in the range" << std::endl;
        return false;
    }

// INIT KINEMATICS - WANTED VELOCITY VECTOR
    (*m_wantedVelocityVector)(0,0) = wanted_distance * cos(DEG_TO_RAD * wanted_angle)*(int)MSTEP/wanted_mstep;
    (*m_wantedVelocityVector)(1,0) = wanted_distance * sin(DEG_TO_RAD * wanted_angle)*(int)MSTEP/wanted_mstep;
    (*m_wantedVelocityVector)(2,0) = DEG_TO_RAD * wanted_rotation*(int)MSTEP/wanted_mstep;

// COMPUTE KINEMATICS
    (*m_wheelAngularSpeedVector) = (*m_inverseJacobianKinematicsMatrix) * (*m_wantedVelocityVector);

    double normWheelAngularSpeedVector;
    if(!Matrix::normVector(*m_wheelAngularSpeedVector, &normWheelAngularSpeedVector))
    {
        std::cout << "Error : Matrix incorrectly used as vector" << std::endl;
        return false;
    }

    double steps_factor = wanted_mstep * TICS_PER_ROTATION / (2 * PI);
    double speed_factor = wanted_mstep * wanted_speed * TICS_PER_ROTATION * WHEEL_RADIUS * 1.0 
        / (SPEED_TO_RPM_FACTOR * SPEED_CORRECTION_FACTOR * normWheelAngularSpeedVector);

// SEND DATA TO MOTORS
    for(int i = 0; i < N_MOTOR; i++)
    {
        m_motors[i]->Go(static_cast<int>((*m_wheelAngularSpeedVector)(i,0) * speed_factor), 
            static_cast<int>(std::abs((*m_wheelAngularSpeedVector)(i,0)) * steps_factor));
    }

    return true;
}


void Robot::Move(void)
{
    double wanted_distance, wanted_angle, wanted_rotation, wanted_speed, wanted_mstep;
    
    bool result = false;
    
    result = false;
    while(!result)
    {
        std::cout << "Distance (mm): ";
        result = static_cast<bool>(std::cin >> wanted_distance);
        std::cout << wanted_distance << std::endl;
        if(!result)
        {
            std::cout << "Not a double. Retry." << std::endl;
            std::cin.clear(); // Clear the error flag on cin
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
        }
    }
  
    result = false;
    while(!result)
    {
        std::cout << "Angle (°): ";
        result = static_cast<bool>(std::cin >> wanted_angle);
        std::cout << wanted_angle << std::endl;
        if(!result)
        {
            std::cout << "Not a double. Retry." << std::endl;
            std::cin.clear(); // Clear the error flag on cin
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
        }
    }

    result = false;
    while(!result)
    {
        std::cout << "Rotation (°): ";
        result = static_cast<bool>(std::cin >> wanted_rotation);
        std::cout << wanted_rotation << std::endl;
        if(!result)
        {
            std::cout << "Not a double. Retry." << std::endl;
            std::cin.clear(); // Clear the error flag on cin
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
        }
    }

    result = false;
    while(!result)
    {
        std::cout << "Speed (mm/s) [ " << RPM_MINIMUM << " ; " << RPM_MAXIMUM << " ] : ";
        std::cin >> wanted_speed;
        std::cout << wanted_speed << std::endl;
        if(RPM_MINIMUM < wanted_speed && wanted_speed < RPM_MAXIMUM)
        {
            result = true;
        }
        else
        {
            std::cout << "Incorrect speed" << std::endl;
            std::cin.clear(); // Clear the error flag on cin
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
        }
    }
    
    result = false;
    while(!result)
    {
        std::cout << "MStep: [ " << MSTEP_MINIMUM << " ; " << MSTEP_MAXIMUM << " ] : ";
        std::cin >> wanted_mstep;
        std::cout << wanted_mstep << std::endl;
        if(MSTEP_MINIMUM < wanted_mstep && wanted_mstep < MSTEP_MAXIMUM)
        {
            result = true;
        }
        else
        {
            std::cout << "Incorrect MStep" << std::endl;
            std::cin.clear(); // Clear the error flag on cin
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
        }
    }
    std::cout << std::endl;


    Move(wanted_distance, wanted_angle, wanted_rotation, wanted_speed, wanted_mstep);

    Move(); // Callback for next movement
}

bool Robot::Calibrate()
{
    bool success = true;
    for(const auto& motor : m_motors)
    {
        if(!motor->Calibrate())
        {
            success = false;
        }
    }
    return success;
}

bool Robot::SetPID(uint16_t kp, uint16_t ki, uint16_t kd)
{
    bool success = true;
    for(int i = 0; i < N_MOTOR; i++)
    {
        if (!m_motors[i]->SetPID(kp,ki,kd))     //(Default Kp is 0x650, 0x1, 0x650).
        {
            success = false;
        }
    }
    return success;
}

bool Robot::SetACC(uint16_t ACC)
{
    bool success = true;
    for(int i = 0; i < N_MOTOR; i++)
    {
        if (!m_motors[i]->SetACC(ACC))    //(Default ACC is 0x11e)
        {
            success = false;
        }
    }

    printMutex.lock();
    if(success)
    {
        std::cout << "Successfully set the acceleration\n" << std::endl;
    }
    else 
    {
        std::cout << "Problem setting the acceleration\n" << std::endl;
    }
    printMutex.unlock();
    return success;
}

bool Robot::SetMStep(uint8_t mStep)
{
    bool success = true;
    for(int i = 0; i < N_MOTOR; i++)
    {
        if (!m_motors[i]->SetMStep(mStep))    
        {
            success = false;
        }
    }

    return success;
}

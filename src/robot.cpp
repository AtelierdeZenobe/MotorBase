#include "robot.h"
#include <istream>
#include <bitset>
#include <iomanip>
// Kinematics (p29) : https://pure.tue.nl/ws/portalfiles/portal/4274124/612987.pdf
// Kinematics (more explanations) : https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf


Robot::Robot(EventQueue* EVqueue) : m_EVqueue(EVqueue)
{
    // InitializeMotorbase();
    InitializeMotorbase();
}

Robot::~Robot()
{
    for(int i = 0; i < N_MOTOR; i++)
    {
        delete m_motors[i]; //delete[] ?
    }
}

bool Robot::InitializeMotorbase(void)
{
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
    SetMStep(static_cast<uint8_t>(MSTEP)); //Conversion is OK cos 0 <= MSTEP <256

    return true;
}

bool Robot::Move(const double& wanted_distance, const double& wanted_angle, const double& wanted_rotation, 
    double& wanted_speed, const double& wanted_mstep)
{
    const double r = static_cast<double>(WHEEL_RADIUS);
    const double R = static_cast<double>(ROBOT_RADIUS);

    double wanted_rotational_speed = wanted_distance != 0 ? wanted_rotation * DEG_TO_RAD * wanted_speed / wanted_distance : 1;
    // deg * (rad/deg) * (mm/s) / mm = rad/s

    if(wanted_distance == 0) wanted_speed = 0;

    std::vector<double> W(3, 0.0);
    for(int i=0; i<3; i++)
    {
        W[i] =  (-sin(theta+a[i])*wanted_speed*cos(wanted_angle*DEG_TO_RAD) // mm/s
                +cos(theta+a[i])*wanted_speed*sin(wanted_angle*DEG_TO_RAD)
                +R*wanted_rotational_speed) // mm * rad/s
                /r; 
        //printMutex.lock();
        //std::cout << "WHeel " << i << " angular speed: " << W[i] << " rad/s" << std::endl;
        //LOG("WHeel %d angular speed: %f rad/s \n", i, W[i]);
        //printMutex.unlock();
    }

    // The number of ticks to rotate for each wheel
    std::vector<double> ticks(3,0.0);
    for(int i=0; i<3; ++i)
    {
        // The wheel only participate in its plane, and simply drifts on the perpendicular direction
        // The distance that the wheel will travel is the combination of its rotation and drift

        // Portion of D in the wheel rotation direction
        // Portion due to translation
        ticks[i] = - wanted_distance * sin (theta + a[i] - wanted_angle*DEG_TO_RAD) * TICS_PER_ROTATION / (2 * PI * r);

        // Portion due to rotation
        ticks[i] += wanted_rotation*DEG_TO_RAD * R / (r) * TICS_PER_ROTATION / (2*PI);

        ticks[i] = std::abs(ticks[i]);
        // printMutex.lock();
        // std::cout << "Motor " << i << " goes for " << ticks[i] << " ticks." << std::endl;
        // printMutex.unlock();
    }

    for(size_t i = 0; i<N_MOTOR; ++i)
    {
        // Convert rotational speed to motor instruction (motor datasheet)
        
        const double Wrpm = W[i]*(double)60/(2*PI);
        const double speed = Wrpm * (double)TICS_PER_ROTATION * (double)MSTEP / 30000;
        // printMutex.lock();
        // std::cout << "Speed of motor " << i << ": " << speed << "(Servo42C speed)" << std::endl;
        // printMutex.unlock();
        // Convert to uint8_t for sending
        uint8_t sign_bit = (speed < 0) ? 0x80 : 0x00;
        uint8_t abs_speed = (uint8_t)round(fabs(speed));
        uint8_t uiSpeed = sign_bit | (abs_speed & 0x7F);
        // printMutex.lock();
        // std::cout << "To motor: " << bitset<8>(uiSpeed) << std::endl;
        // printMutex.unlock();
        m_motors[i]->Go(uiSpeed, static_cast<uint32_t>(ticks[i] * MSTEP));
        
    }
    return true;
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
    if(!success)
    {
        std::cerr << "Problem setting the acceleration\n" << std::endl;
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

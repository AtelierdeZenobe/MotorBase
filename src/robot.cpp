#include "robot.h"
#include <istream>
#include <bitset>
#include <iomanip>
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

    SetMStep(static_cast<uint8_t>(MSTEP)); //COnversion is OK cos 0 <= MSTEP <256

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
    SetMStep(static_cast<uint8_t>(MSTEP)); //COnversion is OK cos 0 <= MSTEP <256

    return true;
}

std::vector<double> dotProduct(const std::vector<std::vector<double>>& K, const std::vector<double>& U) {

    // Result vector for the 1x3 output
    std::vector<double> result(3, 0.0);

    // Perform the dot product calculation
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            result[i] += K[i][j] * U[j];
        }
    }

    return result;
}

bool Robot::Move(const double& wanted_distance, const double& wanted_angle, const double& wanted_rotation, 
    const double& wanted_speed, const double& wanted_rotational_speed, const double& wanted_mstep)
{
    const double r = static_cast<double>(WHEEL_RADIUS);
    const double R = static_cast<double>(ROBOT_RADIUS);
    // The global robot mouvements
    std::vector<double> U = {wanted_speed * cos(wanted_angle), wanted_speed * sin(wanted_angle), wanted_rotational_speed};

    // Computed using wolfram alpha
    std::vector<std::vector<double>> invK =
    {{-1/sqrt(3), (double)1/3, R/3}, {0, -(double)2/3, R/3}, {(double)1/sqrt(3), (double)1/3, R/3}};

    // The wheel linear speeds V = invK * U
    std::vector<double> V = dotProduct(invK,U);

    // Wheel rotational speed W = V/r
    std::vector<double> W(3, 0.0);
    for(int i=0; i<3; i++)
    {
        W[i]=V[i]/r;
    }

    // The number of ticks to rotate for each wheel
    std::vector<double> ticks(3,0.0);

    for(int i=0; i<3; ++i)
    {
        // The wheel only participate in its plane, and simply drifts on the perpendicular direction
        // The distance that the wheel will travel is the combination of its rotation and drift

        // Portion of D in the wheel rotation direction
        // Portion due to translation
        ticks[i] = wanted_distance * sin (θ + a[i] - wanted_angle) * TICS_PER_ROTATION / (2 * PI * r);

        // Portion due to rotation
        ticks[i] += wanted_rotation * R / (r) * TICS_PER_ROTATION / (2*PI);

        // Take the absolute
        ticks[i] = std::abs(ticks[i]);
    }

    for(size_t i = 0; i<N_MOTOR; ++i)
    {
        // Convert rotational speed to motor instruction (motor datasheet)
        printMutex.lock();
        const double Wrpm = W[i]*(double)60/(2*PI);
        const double speed = Wrpm * (double)TICS_PER_ROTATION * (double)MSTEP / 30000;
        std::cout << "Speed of motor " << i << ": " << speed << std::endl;
        // Convert to uint8_t for sending
        uint8_t sign_bit = (speed < 0) ? 0x80 : 0x00;
        uint8_t abs_speed = (uint8_t)round(fabs(speed));
        uint8_t uiSpeed = sign_bit | (abs_speed & 0x7F);
        std::cout << "To motor: " << bitset<8>(uiSpeed) << std::endl;
        m_motors[i]->Go(uiSpeed, static_cast<uint32_t>(ticks[i] * MSTEP));
        printMutex.unlock();
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

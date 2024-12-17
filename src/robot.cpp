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

    SetMStep(static_cast<uint8_t>(MSTEP)); //COnversion is OK cos 0 <= MSTEP <256

}

bool invertMatrix(const float input[3][3], float output[3][3]) {
    float determinant = 0;

    // Calculate the determinant of the matrix
    for (int i = 0; i < 3; i++) {
        determinant += (input[0][i] * (input[1][(i + 1) % 3] * input[2][(i + 2) % 3] - input[1][(i + 2) % 3] * input[2][(i + 1) % 3]));
    }

    if (determinant == 0) {
        std::cerr << "Matrix is not invertible (determinant is 0)." << std::endl;
        return false;
    }

    float invDet = 1.0f / determinant;

    // Calculate the inverse matrix using the formula
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            output[j][i] = invDet * (input[(i + 1) % 3][(j + 1) % 3] * input[(i + 2) % 3][(j + 2) % 3] - input[(i + 1) % 3][(j + 2) % 3] * input[(i + 2) % 3][(j + 1) % 3]);
        }
    }

    return true;
}

bool Robot::InitializeMotorbase(void)
{
    return true;
}

bool Robot::Move(const int& wanted_distance, const int& wanted_angle, const int& wanted_rotation, 
    const int& wanted_speed, const int& wanted_mstep)
{
    /*
    phi 1 is roue 3
    */
    // Derived from wanted_angle and wanted_speed
    //TODO: wanted angular rotation as additional parameter ?

    float vx=wanted_speed*cos(wanted_angle*DEG_TO_RAD), vy=wanted_speed*sin(wanted_angle*DEG_TO_RAD), w=(float)wanted_speed/ROBOT_RADIUS;

    std::cout << "wanted speed" << wanted_speed << std::endl;
    std::cout << "w" << w << std::endl;
    if(wanted_distance==0)
    {
        vx = 0;
        vy = 0;
    }
    
    if(wanted_rotation==0)
    {
        w=0;
    }
    // TODO: compute the cos and sines only once at init
    // TODO: correct types to avoid conversion erros
    float θ = 60*DEG_TO_RAD; // angle to the first wheel (number 3)
    float 𝛼[3] = {0,120*DEG_TO_RAD,240*DEG_TO_RAD};
    float beta = wanted_angle * DEG_TO_RAD;

    // TODO: pre-compute sin(theta) and cos(theta)
    //https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf
    float w1 = (-sin(θ) * cos(θ) * vx + cos(θ) * cos(θ) * vy + ROBOT_RADIUS * w )/WHEEL_RADIUS;
    float w2 = (-sin(θ + 𝛼[1]) * cos(θ) * vx + cos(θ + 𝛼[1]) * cos(θ) * vy + ROBOT_RADIUS * w )/WHEEL_RADIUS;
    float w3 = (-sin(θ + 𝛼[2]) * cos(θ) * vx + cos(θ + 𝛼[2]) * cos(θ) * vy + ROBOT_RADIUS * w )/WHEEL_RADIUS;

    std::cout << "w1: " << w1 << std::endl;
    std::cout << "w2: " << w2 << std::endl;
    std::cout << "w3: " << w3 << std::endl;

    float t1 = std::abs(wanted_distance * cos(beta) * 200 / (2 * PI * WHEEL_RADIUS * sin(θ)) - wanted_distance * sin(beta) * 200 / (2 * PI * WHEEL_RADIUS * cos(θ)) + wanted_rotation * ROBOT_RADIUS * 200 / (WHEEL_RADIUS * 360));
    float t2 = std::abs(- wanted_distance * sin(beta) * 200 / (2 * PI * WHEEL_RADIUS* cos(θ+𝛼[1])) + wanted_rotation * ROBOT_RADIUS * 200 / (WHEEL_RADIUS * 360));
    float t3 = std::abs(wanted_distance * cos(beta) * 200 / (2 * PI * WHEEL_RADIUS * sin(θ+𝛼[2])) - wanted_distance * sin(beta) * 200 / (2 * PI * WHEEL_RADIUS * cos(θ+𝛼[2])) + wanted_rotation * ROBOT_RADIUS * 200 / (WHEEL_RADIUS * 360));

    std::cout << "t1: " << t1 << std::endl;
    std::cout << "t2: " << t2 << std::endl;
    std::cout << "t3: " << t3 << std::endl;

    std::cout << "t1/t2: " << t1/t2 << std::endl;
    std::cout << "v1/v2: " << w1/w2 << std::endl;

    std::cout << "rotation" << wanted_rotation * ROBOT_RADIUS * 200 / (WHEEL_RADIUS * 360) << std::endl;



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

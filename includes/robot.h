#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <cstdint>
#include <iostream>
#include <cmath>
#include <vector>
#include "matrix.h"
#include "motor.h"
#include "constants.h"

/**
 * @brief 
 *
 */
class Robot 
{
    public:

        /**  
         * @brief  
         * 
         * @param EVqueue :
         */
        Robot(EventQueue *EVqueue);
        Robot() = delete;
        ~Robot();

        /**
         * @brief
         */
        bool InitializeMotorbase(void);

        /**
         * @brief 
         *
         * @param wanted_distance : travel distance in mm
         * @param wanted_angle : travel angle in degrees
         * @param wanted_rotation : rotation angle in degrees
         * @param wanted_speed : 
         * @param wanted_mstep = 128 :
         */
        bool Move(const int& wanted_distance, const int& wanted_angle, const int& wanted_rotation, 
            int wanted_speed, const int& wanted_mstep = 128);

        /**
         * @brief Recalibrate each motor to rectify any abnormal behaviour occuring
         */
        //bool CalibrateMotors(void);

    private:   
        EventQueue* m_EVqueue;
        Motor* m_motors[N_MOTOR];

// KINEMATICS Main equation in pdf : [Theta_dot] = [Jinv] x [u_dot]
// KINEMATICS Main equation here : [m_wheelAngularSpeedVector] = [m_inverseJacobianKinematicsMatrix] x [wantedVelocityVector]
        Matrix* m_wheelAngularSpeedVector;
        Matrix* m_inverseJacobianKinematicsMatrix;
        Matrix* m_wantedVelocityVector;
};

#endif
#pragma once

#include <cstdint>
#include <iostream>
#include <cmath>
#include <istream>
#include <vector>
#include "matrix.h"
#include "motor.h"
#include "constants.h"

/**
 * @brief Represents a 3-omniwheel robot
 */
class Robot 
{
    public:
        Robot() = delete;
        ~Robot();

        /**  
         * @brief Main constructor used to instantiate a robot
         *  
         * @param EVqueue :
         */
        Robot(EventQueue *EVqueue);

        /**
         * @brief Function used to initialize all motorbase components
         *
         * @attention Must be used right after the constructor
         */
        bool InitializeMotorbase(void);

        /**
         * @brief Function used to move and rotate the robot in any way with given values
         *
         * @param wanted_distance : travel distance in mm
         * @param wanted_angle : travel angle in degrees
         * @param wanted_rotation : rotation angle in degrees
         * @param wanted_speed : speed of the motors in mm/s
         * @param wanted_mstep : mstep of the motors
         */
        bool Move(const int& wanted_distance, const int& wanted_angle, const int& wanted_rotation, 
            const int& wanted_speed, const int& wanted_mstep = 128);

        /**
         * @brief Move the robot using terminal inputs
         */
        void Move(void);


        /**
         * @brief Function used to recalibrate the motors when something is getting wrong with them
         */
        //bool CalibrateMotors(void);
    private:   
        EventQueue* m_EVqueue;
        Motor* m_motors[N_MOTOR];

// KINEMATICS Main equation in the pdf : [Theta_dot] = [Jinv] x [u_dot]
// KINEMATICS Main equation here : [m_wheelAngularSpeedVector] = [m_inverseJacobianKinematicsMatrix] x [wantedVelocityVector]
        Matrix* m_wheelAngularSpeedVector;
        Matrix* m_inverseJacobianKinematicsMatrix;
        Matrix* m_wantedVelocityVector;
};
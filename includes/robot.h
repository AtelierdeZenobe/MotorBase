#pragma once

#include <cstdint>
#include <iostream>
#include <cmath>
#include <istream>
#include <vector>
#include "matrix.h"
#include "motor.h"
#include "constants.h"

//TODO TOREMOVE
#include "functionCodes.h"
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
        bool Move(const double& wanted_distance, const double& wanted_angle, const double& wanted_rotation, 
            double& wanted_speed, const double& wanted_mstep = MSTEP);

        /**
        * @brief Set PID of all robot's motor
        */
            //(Default Kp is 0x650).
            //(Default Ki is 1).
            //(Default Kd is 0x650).
        bool SetPID(uint16_t kp, uint16_t ki, uint16_t kd);


        /**
        * @brief Set ACC of all robot's motor
        */
        //(Default ACC is 0x11e)
        bool SetACC(uint16_t ACC);

        /**
        * @brief Set mStep of all robot's motor
        */
        bool SetMStep(uint8_t mStep);

        /**
         * @brief Calibrate the motors
         */
        bool Calibrate();


    private:   
        EventQueue* m_EVqueue;
        Motor* m_motors[N_MOTOR];

// KINEMATICS Main equation in the pdf : [Theta_dot] = [Jinv] x [u_dot]
// KINEMATICS Main equation here : [m_wheelAngularSpeedVector] = [m_inverseJacobianKinematicsMatrix] x [wantedVelocityVector]
        Matrix* m_wheelAngularSpeedVector;
        Matrix* m_inverseJacobianKinematicsMatrix;
        Matrix* m_wantedVelocityVector;
};
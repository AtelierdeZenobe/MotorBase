


#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <cstdint>
#include <iostream>
#include <cmath>
#include "matrix-master/dist/matrix.h"
#include "motor.h"

const uint8_t tMotors = 3; //MAX 9

template <uint8_t TMotors>
class Robot 
{
    public :
        Robot(EventQueue *EVqueue);
        ~Robot();

        enum class Direction : uint8_t { 
            CounterClockWise = 0,
            ClockWise = 1
        };

        enum class Motors : uint8_t {
            Motor1 = 0, Motor2 = 1, Motor3 = 2, 
            Motor4 = 3, Motor5 = 4, Motor6 = 5, 
            Motor7 = 6, Motor8 = 7, Motor9 = 8
        };

        const uint8_t maxMotors = 9;

        void setMotors(void);
        void setMatrix(float r, float b, float theta, float alpha_1, float alpha_2, float alpha_3); //3 motors
        //void setMatrix(float r, float b, float theta, float alpha_1, float alpha_2, float alpha_3, float alpha_4); //4 motors
        Robot* setVelocity(float v_x, float v_y, float omega);
        Matrix* findPhi(void);
        Direction signPhi(int row);
        uint8_t absPhi(int row);


        //Robot* calculatePhi(void);
        Matrix* getPhi(void);
        //Matrix* getMatrix(void);
        //Matrix* getVelocity(void);

        Motor* getMotor(Robot<TMotors>::Motors motor);


    private :
        const float m_DEG2RAD = 3.141592f / 180.0f;
        const uint8_t m_firstAddress = 0xE1;
        
        EventQueue* m_EVqueue;
        Motor* m_motors[TMotors];
        Matrix* m_matrix;
        Matrix* m_velocity;
        Matrix* m_phi;

        
        //void Move(); //?


};






#endif
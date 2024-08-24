


#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <cstdint>
#include <iostream>
#include <cmath>
#include "matrix-master/dist/matrix.h"
#include "motor.h"

// MaxMotors : maximum number of motors which can be used simultaneoulsy
static const uint8_t MaxMotors = 3;

// DegToRad : constant used to convert angles from degrees to radians
static constexpr float DegToRad = 3.141592f / 180.0f;

class Robot 
{
    public :

        /*  EVqueue : , 
         *  motor : number of motors used,
         *  firstMotorAddress : address of the first motor while next ones are expected to be resulting from an incrementation
         */
        Robot(EventQueue *EVqueue, uint8_t motor, uint8_t firstMotorAddress = 0xE1);
        Robot() = delete;
        ~Robot();


        struct Pos {
            int x;
            int y;
        };

        // Returns current position of the robot
        struct Pos* Pos(void);

        /* Initializes main characteristics of the robot,
         * r : distance between the center of the motorbase and the center of the omniwheel,
         * b : radius of the omniwheels used,
         * theta : angle in degrees between the coordinate robot axes and the first omniwheel,
         * pos_x : current x-position of the robot,
         * pos_y : current y-position of the robot,
         * alpha_1 : angle between the coordinate wheel axes and the first wheel,
         * alpha_2 : angle between the coordinate wheel axes and the second wheel,
         * alpha_3 : angle between the coordinate wheel axes and the third wheel,
         */
        void Initialize(float r, float b, float theta, int pos_x = 0, int pos_y = 0, float alpha_1 = 0.0f, float alpha_2 = 120.0f, float alpha_3 = 240.0f);
  
        bool Calibrate(void);

        /// -----------------------------

        Robot* setVelocity(float v_x, float v_y, float omega);
        Matrix* findPhi(void);
        uint8_t signPhi(int row);
        double absPhi(int row);


        Matrix* getPhi(void);
        //Matrix* getMatrix(void);
        //Matrix* getVelocity(void);


        enum class Direction : uint8_t { 
            CounterClockWise = 0,
            ClockWise = 1
        };

        bool Move(int dest_x, int dest_y);
        bool Move(int dest_x, int dest_y, int v_x, int v_y, int omega);


    private :
        const uint8_t m_firstMotorAddress;
        const uint8_t m_motor;
        
        EventQueue* m_EVqueue;
        Motor* m_motors[MaxMotors];
        Motor* m_currentMotor;

        // m_phi = m_matrix x m_velocity
        Matrix* m_matrix;
        Matrix* m_velocity;
        Matrix* m_phi;

        struct Pos* m_previousPos; // ?
        struct Pos* m_currentPos;

        void initMotors(void);
        void initPos(int pos_x, int pos_y);
        void initMatrix(float r, float b, float theta, float alpha_1 = 0.0f, float alpha_2 = 120.0f, float alpha_3 = 240.0f); //3 motors
};






#endif
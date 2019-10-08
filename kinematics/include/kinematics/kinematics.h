/**
*   @author : koseng (Lintang) & Gabrielle
*   @brief : Three-omniwheel kinematics
*/


#pragma once

#include <ros/ros.h>

#include <armadillo>

#include "kinematics/robot_data.h"

using namespace arma;

typedef colvec3 RobotVel;
typedef colvec3 MotorVel;

class Kinematics{
public:    

    RobotVel forwardKinematics(const MotorVel& _motor_vel);
    MotorVel inverseKinematics(const RobotVel& _robot_vel);

    MotorVel inverseKinematics_2(const RobotVel& _robot_vel);
    RobotVel forwardKinematics_2(const MotorVel& _motor_vel);

    mat33 r_to_m;
    mat33 m_to_r;

    static Kinematics& inst(){
        static Kinematics instance;
        return instance;
    }

private:
    Kinematics();
    ~Kinematics();

    const static double RAD2DEG;
    const static double DEG2RAD;
    const static double SPIN_RATE;
};

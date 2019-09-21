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

    MotorVel inverseKinematics_2();

    mat33 r_to_m;
    mat33 m_to_r;

    mat33 motor;
    RobotVel velocity;
    MotorVel motor_velocity;
    double velocity_phi;

    static Kinematics& inst(){
        static Kinematics instance;
        return instance;
    }

private:
    Kinematics();
    ~Kinematics();

    inline double radToDeg(double rad);
    inline double degToRad(double deg);
};

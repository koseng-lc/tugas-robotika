#include "kinematics/kinematics.h"

const double Kinematics::DEG2RAD(M_PI / 180.0);
const double Kinematics::RAD2DEG(180.0 / M_PI);
const double Kinematics::SPIN_RATE(60);

Kinematics::Kinematics(){
    m_to_r << -1      << .5                 << .5                << endr
           << 0       << -std::sqrt(3) * .5 << std::sqrt(3) * .5 << endr
           << 0.33333 << 0.33333            << 0.33333           << endr;

    m_to_r *= RobotData::WHEEL_RAD;

    r_to_m = inv(m_to_r);
}

Kinematics::~Kinematics(){

}

RobotVel Kinematics::forwardKinematics(const MotorVel& _motor_vel){
    return m_to_r * _motor_vel;
}

MotorVel Kinematics::inverseKinematics(const RobotVel& _robot_vel){
    return r_to_m * _robot_vel;
}

MotorVel Kinematics::inverseKinematics_2(const RobotVel& _robot_vel){
    mat33 motor;
    auto velocity_phi(std::atan2(_robot_vel.at(1), _robot_vel.at(0)));
    motor << 1/(RobotData::WHEEL_RAD*sin(DEG2RAD * (300) - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(DEG2RAD * (300) - velocity_phi)) << 1 << endr
          << 1/(RobotData::WHEEL_RAD*sin(DEG2RAD * (60)  - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(DEG2RAD * (60)  - velocity_phi)) << 1 << endr
          << 1/(RobotData::WHEEL_RAD*sin(DEG2RAD * (180) - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(DEG2RAD * (180) - velocity_phi)) << 1 << endr;
    return motor * _robot_vel / SPIN_RATE;
}

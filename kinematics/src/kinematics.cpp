#include "kinematics/kinematics.h"

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
    return r_to_m *_robot_vel;
}

MotorVel Kinematics::inverseKinematics_2(){
    motor << 1/(RobotData::WHEEL_RAD*sin(degToRad(300) - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(degToRad(300) - velocity_phi)) << 1 << endr
          << 1/(RobotData::WHEEL_RAD*sin(degToRad(60)  - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(degToRad(60)  - velocity_phi)) << 1 << endr
          << 1/(RobotData::WHEEL_RAD*sin(degToRad(180) - velocity_phi)) << 1/(RobotData::WHEEL_RAD*cos(degToRad(180) - velocity_phi)) << 1 << endr;
    return motor;
}

inline double Kinematics::radToDeg(double rad){
    return rad * 180.0 / M_PI;
}

inline double Kinematics::degToRad(double deg){
    return deg * M_PI / 180.0;
}

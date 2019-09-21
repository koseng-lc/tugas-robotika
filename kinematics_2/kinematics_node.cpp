#include "kinematics.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "kinematics_node");

    Kinematics kinematics;

    ros::Rate loop_rate(SPIN_RATE);

    while(ros::ok())
    {
        kinematics.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

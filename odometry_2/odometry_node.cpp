#include "odometry.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "odometry_node");

    Odometry odometry;

    ros::Rate loop_rate(SPIN_RATE);

    while(ros::ok())
    {
        odometry.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

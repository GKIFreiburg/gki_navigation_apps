#include <QApplication>
#include "calibrationmain.h"
#include <ros/ros.h>

bool g_Quit = false;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibrateTwist_client");


    QApplication app(argc, argv);

    ros::NodeHandle nh;

    CalibrationMain w;
    w.show();

    ros::Rate loopRate(100.0);
    while(!g_Quit && ros::ok() && w.isVisible()) {
        ros::spinOnce();
        app.processEvents();

        loopRate.sleep();
    }
    
    return 0;
}


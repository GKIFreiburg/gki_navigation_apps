#ifndef CALIBRATIONMAIN_123_H
#define CALIBRATIONMAIN_123_H

#include <QMainWindow>
#include <QListView>
#include <QStandardItem>
#include <QStandardItemModel>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <calibrate_twist/CalibrateAction.h>

#include <tf/transform_listener.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <move_base_msgs/MoveBaseAction.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


#include <fstream>

using namespace std;

namespace Ui {
class CalibrationMain;
}

typedef actionlib::SimpleActionClient<calibrate_twist::CalibrateAction> Client;

class CalibrationMain : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CalibrationMain(QWidget *parent = 0);
    ~CalibrationMain();
    void doneCB(const actionlib::SimpleClientGoalState& goal, const calibrate_twist::CalibrateResultConstPtr& result);
    
private Q_SLOTS:
    void on_buttonStart_clicked();
    void on_buttonClear_clicked();
    void on_buttonCancel_clicked();

    void on_buttonContStart_clicked();
    void on_buttonContCancel_clicked();

    void on_buttonSendHome_clicked();

private:
    void send_goal(double vx, double vrot, double time);
    void printToFile(const calibrate_twist::CalibrateResultConstPtr& result);
    void printStoreToYAML();

    bool sendHomePositionGoal();
    bool startContinuousRun();

    Ui::CalibrationMain *ui;
    int count;
    QStandardItemModel* listModel;
    Client ac;
    calibrate_twist::CalibrateGoal currentGoal;
    bool goalSucceeded; // if the current goal was achieved successfully or not
    bool vxvrInd;
    double iterations;

    string filename;
    string yamlname;
    bool keepFile;
    std::vector<calibrate_twist::CalibrateGoal> goals;
    std::vector<calibrate_twist::CalibrateResult> results;

    tf::TransformListener* listener; // TF listener
    geometry_msgs::Pose homeTransform; // home position
};

inline bool compareDouble(double x, double y)
{
    return (fabs(x-y)< 0.0000001);
}

#endif // CALIBRATIONMAIN_H

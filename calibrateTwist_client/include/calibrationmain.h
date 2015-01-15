#ifndef CALIBRATIONMAIN_H
#define CALIBRATIONMAIN_H

#include <QMainWindow>
#include <QListView>
#include <QStandardItem>
#include <QStandardItemModel>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <calibrate_twist/CalibrateAction.h>

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
    
private slots:
    void on_buttonStart_clicked();
    void on_buttonClear_clicked();
    void on_buttonCancel_clicked();

private:
    void send_goal(double vx, double vrot, double time);
    void printToFile(const calibrate_twist::CalibrateResultConstPtr& result);

    Ui::CalibrationMain *ui;
    int count;
    QStandardItemModel* listModel;
    Client ac;
    calibrate_twist::CalibrateGoal currentGoal;

    string filename;
    bool keepFile;

};

#endif // CALIBRATIONMAIN_H

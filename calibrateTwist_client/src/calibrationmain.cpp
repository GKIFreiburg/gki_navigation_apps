#include "calibrationmain.h"
#include "ui_calibrationmain.h"


CalibrationMain::CalibrationMain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationMain),
    ac("calibrate_twist", true)
{
    ui->setupUi(this);
    count = 0;
    listModel = new QStandardItemModel();

    ROS_INFO("Waiting for action server to start");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started successfully");
}

CalibrationMain::~CalibrationMain()
{
    delete ui;
}

void CalibrationMain::on_buttonCancel_clicked()
{
    ac.cancelGoal();
}

void CalibrationMain::on_buttonClear_clicked()
{
    listModel->clear();
    ui->listResult->setModel(listModel);
}

void CalibrationMain::on_buttonStart_clicked()
{
    //count++;
    //QString value = QString::number(count);
    //ui->labelStatus->setText("clicked" + value);
    double xSpeed = ui->SpinBoxXSpeed->value();
    double rotSpeed = ui->SpinBoxRotSpeed->value();
    double time = ui->SpinBoxRotSpeed_2->value();

    send_goal(xSpeed, rotSpeed, time);
}


void CalibrationMain::send_goal(double vx, double vrot, double time)
{
    // send a goal to the action
    calibrate_twist::CalibrateGoal goal;
    goal.duration = ros::Duration(time);
    goal.twist_goal.angular.z = vrot;
    goal.twist_goal.angular.x = 0;
    goal.twist_goal.angular.y = 0;
    goal.twist_goal.linear.x = vx;
    goal.twist_goal.linear.y = 0;
    goal.twist_goal.linear.z = 0;


//    if(ac.isServerConnected())
//    {
        ac.sendGoal(goal,
                boost::bind(&CalibrationMain::doneCB, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());
/*    }
    else
    {
        ROS_INFO("Server not connected");
        ui->labelStatus->setText("Server not connected");
        ac.waitForServer(ros::Duration(10));
        if(ac.isServerConnected())
        {
            ROS_INFO("reconnected");
        }
        else
        {
            ROS_INFO("disconnected");
        }
    }
*/
}

void CalibrationMain::doneCB(const actionlib::SimpleClientGoalState& goal, const calibrate_twist::CalibrateResultConstPtr& result)
{
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    QString text = QString::fromStdString(state.toString());
    ui->labelStatus->setText("State: " + text);

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("state is succeeded");
      calibrate_twist::CalibrateResultConstPtr result = ac.getResult();

      QString myString = "vx: " + QString::number(result->calibrated_result.twist.linear.x) + ", " +
                         "vrot: " + QString::number(result->calibrated_result.twist.angular.z);
      QStandardItem* items = new QStandardItem(myString);
      listModel->appendRow(items);
      ui->listResult->setModel(listModel);
    }
}











